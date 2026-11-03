#include "catch2/catch.hpp"
#include "slic3r/GUI/Jobs/PrintJobCompletion.hpp"

using Slic3r::GUI::print_job_id_from_reply;

TEST_CASE("Print completion extracts a string job ID for immediate telemetry matching", "[print-job-completion]")
{
    const std::string observed_job = "1234567890123";
    REQUIRE(print_job_id_from_reply(R"({"job_id":"1234567890123"})") == observed_job);
    REQUIRE(print_job_id_from_reply(R"({"job_id":"different-job"})") != observed_job);
}

TEST_CASE("Print completion preserves numeric job IDs without floating point conversion", "[print-job-completion]")
{
    REQUIRE(print_job_id_from_reply(R"({"job_id":1234567890123})") == "1234567890123");
    REQUIRE(print_job_id_from_reply(R"({"job_id":18446744073709551615})") == "18446744073709551615");
}

TEST_CASE("Print completion ignores missing and malformed job IDs", "[print-job-completion]")
{
    for (const char *reply :
         {"not JSON", "", "null", "[]", "{}", R"({"job_id":null})", R"({"job_id":true})", R"({"job_id":{}})", R"({"job_id":[]})", R"({"job_id":1.5})", R"({"job_id":""})"}) {
        INFO(reply);
        REQUIRE(print_job_id_from_reply(reply).empty());
    }
}
