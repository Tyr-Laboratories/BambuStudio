#include "src/slic3r/Utils/PJarczakLinuxBridge/PJarczakLinuxSoBridgeRpcProtocol.hpp"
#include <iostream>
#include <sstream>

using namespace Slic3r::PJarczakLinuxBridge;
int main()
{
    // Header announces 16 MiB + 1. Reject it before allocating its payload.
    const unsigned char header[] = {0x50, 0x4a, 0x42, 0x52, 1, 0, 0, 0,
        1, 0, 0, 0, 1, 0, 0, 1};
    std::istringstream input(std::string(reinterpret_cast<const char*>(header), sizeof(header)));
    RawRpcFrame frame;
    std::string error;
    if (read_raw_frame(input, frame, error) || frame.payload.capacity() != 0 ||
        error != "frame exceeds size limit") {
        std::cerr << "Oversized frame was not rejected before allocation\n";
        return 1;
    }
    std::ostringstream output;
    if (!write_json_frame(output, RpcFrameType::json_request, 3,
            {{"method", "bridge.ping"}, {"payload", nlohmann::json::object()}}))
        return 2;
    std::istringstream valid(output.str());
    RpcFrame request;
    if (!read_raw_frame(valid, frame, error) || !read_request_frame(frame, request, error) ||
        request.method != "bridge.ping" || request.id != 3)
        return 3;
    return 0;
}
