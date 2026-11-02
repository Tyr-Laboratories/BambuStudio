#define CATCH_CONFIG_RUNNER
#include "catch2/catch.hpp"

#include "slic3r/GUI/Jobs/Job.hpp"

#include <wx/app.h>
#include <wx/dialog.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/utils.h>

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {

class JobTestApp final : public wxApp
{
public:
    bool OnInit() override { return true; }
};

struct ProgressCall {
    enum Kind { Range, Cancel, Progress, Status, Clear, Error } kind;
    std::thread::id thread;
    int value = 0;
    wxString message, description, extra;
};

// This records the UI boundary of the actual production Job. It intentionally
// does not call native widgets off-thread, so regressions fail assertions rather
// than aborting the whole test process in AppKit.
class RecordingProgress final : public Slic3r::ProgressIndicator
{
    mutable std::mutex m_mutex;
    std::vector<ProgressCall> m_calls;
    int m_range = 250;

    void record(ProgressCall::Kind kind, int value = 0, wxString message = {},
                wxString description = {}, wxString extra = {})
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_calls.push_back({kind, std::this_thread::get_id(), value,
                           std::move(message), std::move(description), std::move(extra)});
    }

public:
    std::function<void()> on_main_thread_error;

    std::vector<ProgressCall> calls() const
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_calls;
    }

    std::vector<ProgressCall> calls(ProgressCall::Kind kind) const
    {
        auto result = calls();
        result.erase(std::remove_if(result.begin(), result.end(), [kind](const ProgressCall& call) {
            return call.kind != kind;
        }), result.end());
        return result;
    }

    void clear_percent() override { record(ProgressCall::Clear); }
    void show_error_info(wxString message, int code, wxString description, wxString extra) override
    {
        record(ProgressCall::Error, code, std::move(message), std::move(description), std::move(extra));
        if (wxIsMainThread() && on_main_thread_error)
            on_main_thread_error();
    }
    void set_range(int range) override { m_range = range; record(ProgressCall::Range, range); }
    void set_cancel_callback(CancelFn callback = {}) override { record(ProgressCall::Cancel, bool(callback)); }
    void set_progress(int progress) override { record(ProgressCall::Progress, progress); }
    void set_status_text(const char* text) override { record(ProgressCall::Status, 0, wxString::FromUTF8(text)); }
    int get_range() const override { return m_range; }
};

class DispatchJob final : public Slic3r::GUI::Job
{
    std::function<void(DispatchJob&)> m_work;

    void process() override
    {
        worker_thread = std::this_thread::get_id();
        m_work(*this);
    }
    void finalize() override
    {
        ++finalizations;
        finalization_thread = std::this_thread::get_id();
        Job::finalize();
    }

public:
    DispatchJob(std::shared_ptr<RecordingProgress> progress, std::function<void(DispatchJob&)> work)
        : Job(std::move(progress)), m_work(std::move(work)) {}

    using Job::show_error_info;
    using Job::update_status;
    bool is_print_job() const override { return true; }
    int finalizations = 0;
    std::thread::id worker_thread, finalization_thread;
};

struct EventLoopCleanup {
    ~EventLoopCleanup()
    {
        wxTheApp->ProcessPendingEvents();
        // Destroying a job before delivery intentionally discards its final
        // status event; balance the real start() busy cursor in that test only.
        while (wxIsBusy())
            wxEndBusyCursor();
    }
};

void check_main_thread_calls(const RecordingProgress& progress)
{
    for (const auto& call : progress.calls())
        CHECK(call.thread == std::this_thread::get_id());
}

} // namespace

wxIMPLEMENT_APP_NO_MAIN(JobTestApp);

int main(int argc, char** argv)
{
    // Keep Catch arguments out of wxApp's command-line parsing.
    int wx_argc = 1;
    char* wx_argv[] = {argv[0], nullptr};
    if (!wxEntryStart(wx_argc, wx_argv))
        return 2;
    if (!wxTheApp->CallOnInit()) {
        wxEntryCleanup();
        return 2;
    }
    const int result = Catch::Session().run(argc, argv);
    wxTheApp->OnExit();
    wxEntryCleanup();
    return result;
}

TEST_CASE("Worker errors wait for the GUI thread and retain their payload", "[GUIJob]")
{
    EventLoopCleanup cleanup;
    auto progress = std::make_shared<RecordingProgress>();
    DispatchJob job(progress, [](DispatchJob& job) {
        wxString message = wxString::FromUTF8("Upload failed: pièce");
        wxString description = "The transfer was rejected";
        wxString extra = "controlled test detail";
        job.show_error_info(message, -42, description, extra);
        message = description = extra = "overwritten after queueing";
    });

    job.start();
    REQUIRE(job.join());
    CHECK(job.worker_thread != std::this_thread::get_id());
    CHECK(progress->calls(ProgressCall::Error).empty());
    CHECK(progress->calls(ProgressCall::Status).empty());
    CHECK(progress->calls(ProgressCall::Progress).empty());

    wxTheApp->ProcessPendingEvents();
    const auto errors = progress->calls(ProgressCall::Error);
    REQUIRE(errors.size() == 1);
    CHECK(errors.front().message == wxString::FromUTF8("Upload failed: pièce"));
    CHECK(errors.front().value == -42);
    CHECK(errors.front().description == "The transfer was rejected");
    CHECK(errors.front().extra == "controlled test detail");
    check_main_thread_calls(*progress);
    CHECK(job.finalizations == 1);
}

TEST_CASE("Ordinary job status restores the range and finalizes only once", "[GUIJob]")
{
    EventLoopCleanup cleanup;
    auto progress = std::make_shared<RecordingProgress>();
    DispatchJob job(progress, [](DispatchJob& job) {
        job.update_status(25, "Uploading");
        job.update_status(100, "Uploaded");
    });
    job.start();
    REQUIRE(job.join());
    CHECK(job.finalizations == 0);
    wxTheApp->ProcessPendingEvents();
    wxTheApp->ProcessPendingEvents();

    const auto values = progress->calls(ProgressCall::Progress);
    REQUIRE(!values.empty());
    CHECK(values.front().value == 25);
    CHECK(values.back().value == 250);
    const auto statuses = progress->calls(ProgressCall::Status);
    REQUIRE(statuses.size() == 2);
    CHECK(statuses[0].message == "Uploading");
    CHECK(statuses[1].message == "Uploaded");
    CHECK(progress->get_range() == 250);
    CHECK(progress->calls(ProgressCall::Cancel).back().value == 0);
    CHECK(job.finalizations == 1);
    CHECK(job.finalization_thread == std::this_thread::get_id());
    CHECK(job.is_finalized());
    check_main_thread_calls(*progress);
}

TEST_CASE("Error presentation follows metadata queued by its job", "[GUIJob]")
{
    EventLoopCleanup cleanup;
    auto progress = std::make_shared<RecordingProgress>();
    std::string metadata, metadata_when_error_displayed;
    progress->on_main_thread_error = [&] { metadata_when_error_displayed = metadata; };
    DispatchJob job(progress, [&](DispatchJob& job) {
        job.CallAfter([&] { metadata = "failure diagnostic"; });
        job.show_error_info("Transfer failed", 1, "", "");
    });
    job.start();
    REQUIRE(job.join());
    CHECK(metadata.empty());
    CHECK(progress->calls(ProgressCall::Error).empty());
    wxTheApp->ProcessPendingEvents();
    CHECK(metadata_when_error_displayed == "failure diagnostic");
    check_main_thread_calls(*progress);
}

TEST_CASE("A final transfer error is displayed even after one hundred percent progress", "[GUIJob]")
{
    EventLoopCleanup cleanup;
    auto progress = std::make_shared<RecordingProgress>();
    DispatchJob job(progress, [](DispatchJob& job) {
        job.update_status(100, "Waiting for printer acknowledgment");
        job.show_error_info("Printer rejected the job", -7, "Rejected", "");
    });
    job.start();
    REQUIRE(job.join());
    wxTheApp->ProcessPendingEvents();
    const auto errors = progress->calls(ProgressCall::Error);
    REQUIRE(errors.size() == 1);
    CHECK(errors.front().message == "Printer rejected the job");
    CHECK(errors.front().value == -7);
    CHECK(job.finalizations == 1);
    check_main_thread_calls(*progress);
}

TEST_CASE("Destroying a completed job discards its undelivered error callbacks", "[GUIJob]")
{
    EventLoopCleanup cleanup;
    auto progress = std::make_shared<RecordingProgress>();
    auto job = std::make_unique<DispatchJob>(progress, [](DispatchJob& job) {
        job.show_error_info("Obsolete dialog error", -9, "", "");
    });
    job->start();
    REQUIRE(job->join());
    job.reset();
    wxTheApp->ProcessPendingEvents();
    CHECK(progress->calls(ProgressCall::Error).empty());
    check_main_thread_calls(*progress);
}

TEST_CASE("A delivered worker error can resize a native dialog safely", "[GUIJob]")
{
    EventLoopCleanup cleanup;
    wxDialog dialog(nullptr, wxID_ANY, "Job dispatch regression");
    auto* label = new wxStaticText(&dialog, wxID_ANY, "Ready");
    auto* sizer = new wxBoxSizer(wxVERTICAL);
    sizer->Add(label, 0, wxALL, 8);
    dialog.SetSizerAndFit(sizer);
    const int initial_height = dialog.GetClientSize().GetHeight();

    auto progress = std::make_shared<RecordingProgress>();
    progress->on_main_thread_error = [&] {
        label->SetLabel("Transfer failed\nDetails are available\nPlease try again");
        dialog.Layout();
        dialog.Fit();
    };
    DispatchJob job(progress, [](DispatchJob& job) {
        job.show_error_info("Transfer failed", -3, "", "");
    });
    job.start();
    REQUIRE(job.join());
    CHECK(dialog.GetClientSize().GetHeight() == initial_height);
    wxTheApp->ProcessPendingEvents();
    CHECK(dialog.GetClientSize().GetHeight() > initial_height);
    check_main_thread_calls(*progress);
}
