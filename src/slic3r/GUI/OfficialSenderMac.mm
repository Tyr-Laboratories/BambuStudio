#include "OfficialSender.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>
#import <Cocoa/Cocoa.h>
#import <Security/Security.h>

namespace Slic3r::GUI::OfficialSender {

fs::path application_path(const fs::path &resources)
{
    return resources.parent_path() / "Helpers" / "BambuStudio.app";
}

bool is_configured(const fs::path &resources)
{
    if (fs::exists(application_path(resources))) return true;
    try {
        std::ifstream stream(resources / "TyrFork" / "release.json");
        nlohmann::json manifest;
        stream >> manifest;
        return manifest.contains("official_sender");
    } catch (...) { return false; }
}

void verify_application(const fs::path &app)
{
    @autoreleasepool {
        NSString *path = [NSString stringWithUTF8String:app.string().c_str()];
        if (!path || !fs::is_regular_file(app / "Contents" / "MacOS" / "BambuStudio"))
            throw std::runtime_error("The bundled official Bambu Studio application is missing. Reinstall the Tyr release.");
        SecStaticCodeRef code = nullptr;
        SecRequirementRef requirement = nullptr;
        OSStatus status = SecStaticCodeCreateWithPath((CFURLRef)[NSURL fileURLWithPath:path], kSecCSDefaultFlags, &code);
        if (status == errSecSuccess)
            status = SecRequirementCreateWithString(CFSTR("anchor apple generic and identifier \"com.bambulab.bambu-studio\" and certificate leaf[subject.OU] = \"T3UBR9Y3B2\""), kSecCSDefaultFlags, &requirement);
        if (status == errSecSuccess)
            status = SecStaticCodeCheckValidity(code, kSecCSStrictValidate | kSecCSCheckNestedCode | kSecCSCheckAllArchitectures, requirement);
        if (requirement) CFRelease(requirement);
        if (code) CFRelease(code);
        if (status != errSecSuccess)
            throw std::runtime_error("The bundled sender's Bambu signature could not be verified. Reinstall the Tyr release.");
    }
}

void launch(const fs::path &app, const fs::path &sender_data, const fs::path &job)
{
    @autoreleasepool {
        // Launch the exact signed executable. The official app's own single-instance
        // protocol forwards subsequent job paths to its existing GUI process.
        NSTask *task = [[NSTask alloc] init];
        const auto executable = app / "Contents" / "MacOS" / "BambuStudio";
        [task setExecutableURL:[NSURL fileURLWithPath:[NSString stringWithUTF8String:executable.string().c_str()]]];
        NSMutableArray<NSString *> *arguments = [NSMutableArray array];
        for (const auto &argument : launch_arguments(sender_data, job))
            [arguments addObject:[NSString stringWithUTF8String:argument.c_str()]];
        [task setArguments:arguments];
        [task setCurrentDirectoryURL:[NSURL fileURLWithPath:[NSString stringWithUTF8String:sender_data.string().c_str()]]];
        [task setStandardOutput:[NSFileHandle fileHandleWithNullDevice]];
        [task setStandardError:[NSFileHandle fileHandleWithNullDevice]];
        NSError *error = nil;
        const bool started = [task launchAndReturnError:&error];
        std::string reason = started ? "" : "Could not open the official print sender.";
        [task release];
        if (!started) throw std::runtime_error(reason);
    }
}

} // namespace Slic3r::GUI::OfficialSender
