#pragma once
#include "bambu_networking.hpp"

namespace Slic3r::TyrCloudSendHelper {
bool available(const std::string &resources_dir);
// Obtain a cloud-only camera session for the existing native video player.
// camera_request retains Studio's device|firmware|protocols argument.
std::string camera_url(const std::string &session_file, const std::string &expected_user_id,
                       const std::string &country, const std::string &resources_dir,
                       const std::string &data_dir, const std::string &camera_request,
                       BBL::WasCancelledFn cancelled = {});
// The opaque SDK session is copied into a private, disposable per-job directory.
// The helper never replaces the native account or device connection.
int send(const BBL::PrintParams &params,
         const std::string      &session_file,
         const std::string      &expected_user_id,
         const std::string      &country,
         const std::string      &resources_dir,
         const std::string      &data_dir,
         BBL::OnUpdateStatusFn   update,
         BBL::WasCancelledFn     cancelled,
         BBL::OnWaitFn           wait);
// Read-only cloud authentication check; never submits a job or controls a printer.
void verify_account(
    const std::string &session_file, const std::string &expected_user_id, const std::string &country, const std::string &resources_dir, const std::string &data_dir);
// Exercises the same target preparation as send, including read-only printer
// status requests. Never submits a job or sends printer control commands.
void verify_printer(const std::string &session_file,
                    const std::string &expected_user_id,
                    const std::string &country,
                    const std::string &resources_dir,
                    const std::string &data_dir,
                    const std::string &printer_id);
} // namespace Slic3r::TyrCloudSendHelper
