#pragma once
#include "bambu_networking.hpp"

namespace Slic3r::TyrCloudSendHelper {
bool available(const std::string &resources_dir);
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
} // namespace Slic3r::TyrCloudSendHelper
