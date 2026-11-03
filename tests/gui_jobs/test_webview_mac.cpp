#include "catch2/catch.hpp"
#include "slic3r/GUI/Widgets/WebView.hpp"

#include <wx/app.h>
#include <wx/evtloop.h>
#include <wx/frame.h>
#include <wx/osx/webview_webkit.h>
#include <wx/timer.h>

namespace {
// Keep native WebKit handler registration and document injection real. Trap
// only its synchronous script boundary: a regression must fail an assertion,
// rather than hanging the test runner when an off-screen WebContent is paused.
class ObservedWebKit final : public wxWebViewWebKit
{
public:
    mutable int synchronous_scripts = 0;
    bool RunScript(const wxString&, wxString* = nullptr) const override
    {
        ++synchronous_scripts;
        return true;
    }
};
}

TEST_CASE("A hidden Mac webview registers its message bridge without waiting for JavaScript", "[WebView][macOS]")
{
    wxFrame frame(nullptr, wxID_ANY, "Local webview regression test", wxDefaultPosition, wxSize(240, 120));
    auto* view = new ObservedWebKit;
    view->SetNonPersistentWebsiteDataStore(true);
    REQUIRE(view->Create(&frame, wxID_ANY, "about:blank"));
    REQUIRE_FALSE(frame.IsShown());

    REQUIRE(WebView::RegisterScriptMessageHandler(view));
    CHECK(view->synchronous_scripts == 0);

    // Revealing and navigating the hidden view must retain the bridge. This
    // uses only an in-memory local page and the real WebKit message event.
    wxGUIEventLoop loop;
    bool received = false;
    view->Bind(wxEVT_WEBVIEW_SCRIPT_MESSAGE_RECEIVED, [&](wxWebViewEvent& event) {
        if (event.GetString() == "local-bridge-ready") {
            received = true;
            if (loop.IsRunning()) loop.Exit();
        }
    });
    wxTimer timeout(&frame);
    frame.Bind(wxEVT_TIMER, [&](wxTimerEvent&) { if (loop.IsRunning()) loop.Exit(); });
    frame.CallAfter([&] {
        frame.Show();
        view->SetPage("<html><body><script>window.wx.postMessage('local-bridge-ready');</script></body></html>", "file:///");
    });
    timeout.StartOnce(5000);
    loop.Run();
    timeout.Stop();
    CHECK(received);
    CHECK(view->synchronous_scripts == 0);
}

TEST_CASE("A Mac webview exposes its message bridge to an already loaded document", "[WebView][macOS]")
{
    wxFrame frame(nullptr, wxID_ANY, "Local webview regression test", wxDefaultPosition, wxSize(240, 120));
    auto* view = new ObservedWebKit;
    view->SetNonPersistentWebsiteDataStore(true);
    REQUIRE(view->Create(&frame, wxID_ANY, "about:blank"));

    const wxString document_url = "file:///current-document.html";
    wxGUIEventLoop loop;
    bool loaded = false, registered = false, received = false;
    view->Bind(wxEVT_WEBVIEW_LOADED, [&](wxWebViewEvent& event) {
        if (event.GetURL() == document_url && !loaded) {
            loaded = true;
            registered = WebView::RegisterScriptMessageHandler(view);
        }
    });
    view->Bind(wxEVT_WEBVIEW_SCRIPT_MESSAGE_RECEIVED, [&](wxWebViewEvent& event) {
        if (event.GetString() == "current-document-ready") {
            received = true;
            if (loop.IsRunning()) loop.Exit();
        }
    });
    wxTimer timeout(&frame);
    frame.Bind(wxEVT_TIMER, [&](wxTimerEvent&) { if (loop.IsRunning()) loop.Exit(); });
    frame.CallAfter([&] {
        frame.Show();
        // The page starts before registration. A document-start script added
        // later cannot help it: delivery requires the asynchronous injection
        // into this already loaded document, without navigating again.
        view->SetPage(
            "<html><body><script>"
            "const bridgeTimer = setInterval(() => {"
            "if (window.wx) { clearInterval(bridgeTimer); window.wx.postMessage('current-document-ready'); }"
            "}, 20);"
            "</script></body></html>", document_url);
    });
    timeout.StartOnce(5000);
    loop.Run();
    timeout.Stop();
    CHECK(loaded);
    CHECK(registered);
    CHECK(received);
    CHECK(view->synchronous_scripts == 0);
}
