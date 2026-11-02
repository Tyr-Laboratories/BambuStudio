#include <catch2/catch.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/Preset.hpp"
#include <boost/filesystem/operations.hpp>
#include <fstream>

using namespace Slic3r;

// Characterize the model-free export path used when a signed sender must open
// one selected plate from an already-sliced, multi-plate project. No slicing or
// networking occurs here; the G-code payload must survive import/export intact.
TEST_CASE("Selected sliced plate export preserves payload and print metadata", "[3mf][sender]")
{
    const auto directory = boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("sender-plates-%%%%-%%%%");
    boost::filesystem::create_directories(directory);
    const auto old_data = data_dir(), old_temporary = temporary_dir();
    set_data_dir(directory.string());
    set_temporary_dir(directory.string());
    struct Cleanup {
        boost::filesystem::path directory;
        std::string data, temporary;
        ~Cleanup() {
            set_data_dir(data); set_temporary_dir(temporary);
            boost::system::error_code ignored; boost::filesystem::remove_all(directory, ignored);
        }
    } cleanup{directory, old_data, old_temporary};
    const std::string first_bytes = "; plate one\nG1 X1.234 Y5.678 E0.001\n";
    const std::string second_bytes = "; plate two: preserve exact widths and moves\r\nG1 X9.876 Y5.432 E0.00012345\r\n";
    const auto first_file = directory / "first.gcode", second_file = directory / "second.gcode";
    std::ofstream(first_file.string(), std::ios::binary) << first_bytes;
    std::ofstream(second_file.string(), std::ios::binary) << second_bytes;
    Model source;
    DynamicPrintConfig config;
    for (const auto &key : FullPrintConfig::defaults().keys())
        config.option(key, true);
    config.set_deserialize_strict({{"printer_model", "Bambu Lab H2S"}, {"nozzle_diameter", "0.4"},
        {"filament_type", "PA-CF"}, {"filament_colour", "#164B35"}, {"filament_ids", "fixture-id"}});
    PlateData first, second;
    first.plate_index = 0; first.plate_name = "First"; first.gcode_file = first_file.string(); first.is_sliced_valid = true;
    second.plate_index = 1; second.plate_name = "Second"; second.gcode_file = second_file.string(); second.is_sliced_valid = true;
    for (auto *plate : {&first, &second}) {
        plate->printer_model_id = "N7";
        plate->nozzle_diameters = "0.4";
        FilamentInfo filament;
        filament.id = 0; filament.type = "PA-CF"; filament.color = "#164B35";
        filament.filament_id = "fixture-id"; filament.used_g = 7.5f;
        plate->slice_filaments_info.push_back(filament);
    }
    const std::string original_file = (directory / "all.gcode.3mf").string();
    StoreParams all;
    all.path = original_file.c_str(); all.model = &source; all.config = &config;
    all.plate_data_list = {&first, &second};
    all.strategy = SaveStrategy::Silence | SaveStrategy::SkipModel | SaveStrategy::WithGcode | SaveStrategy::SkipAuxiliary;
    REQUIRE(store_bbs_3mf(all));

    Model imported;
    DynamicPrintConfig imported_config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    PlateDataPtrs imported_plates;
    std::vector<Preset*> presets;
    struct ImportedCleanup {
        PlateDataPtrs &plates; std::vector<Preset*> &presets;
        ~ImportedCleanup() { for (auto *p : plates) delete p; for (auto *p : presets) delete p; }
    } imported_cleanup{imported_plates, presets};
    bool is_bambu = false; Semver version;
    REQUIRE(load_bbs_3mf(original_file.c_str(), &imported_config, &substitutions, &imported,
        &imported_plates, &presets, &is_bambu, &version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE(imported.objects.empty());
    REQUIRE(imported_plates.size() == 2);
    REQUIRE(boost::filesystem::is_regular_file(imported_plates[1]->gcode_file));
    CHECK(imported_config.opt_string("printer_model") == "Bambu Lab H2S");
    CHECK(imported_config.option<ConfigOptionStrings>("filament_type")->get_at(0) == "PA-CF");

    // PartPlateList::store_to_3mf_structure omits other plates' G-code when a
    // concrete plate index is requested. Retain their metadata/index positions.
    imported_plates[0]->gcode_file.clear();
    imported_plates[0]->is_sliced_valid = false;
    imported_plates[1]->is_sliced_valid = true;
    // Plater::export_3mf resolves these from the imported printer preset.
    imported_plates[1]->printer_model_id = "N7";
    imported_plates[1]->nozzle_diameters = "0.4";
    const std::string selected_file = (directory / "selected.gcode.3mf").string();
    StoreParams selected;
    selected.path = selected_file.c_str(); selected.model = &imported; selected.config = &imported_config;
    selected.plate_data_list = imported_plates; selected.export_plate_idx = 1; selected.strategy = all.strategy;
    REQUIRE(store_bbs_3mf(selected));

    mz_zip_archive archive;
    mz_zip_zero_struct(&archive);
    REQUIRE(open_zip_reader(&archive, selected_file));
    struct ArchiveCleanup { mz_zip_archive &archive; ~ArchiveCleanup() { close_zip_reader(&archive); } } archive_cleanup{archive};
    CHECK(mz_zip_reader_locate_file(&archive, "Metadata/plate_1.gcode", nullptr, 0) == -1);
    const auto extract = [&](const char *name) {
        size_t size = 0;
        void *bytes = mz_zip_reader_extract_file_to_heap(&archive, name, &size, 0);
        REQUIRE(bytes != nullptr);
        std::string result(static_cast<const char *>(bytes), size); mz_free(bytes); return result;
    };
    CHECK(extract("Metadata/plate_2.gcode") == second_bytes);
    const auto slice_info = extract("Metadata/slice_info.config");
    CHECK(slice_info.find("printer_model_id\" value=\"N7\"") != std::string::npos);
    CHECK(slice_info.find("nozzle_diameters\" value=\"0.4\"") != std::string::npos);
    CHECK(slice_info.find("PA-CF") != std::string::npos);
    CHECK(slice_info.find("fixture-id") != std::string::npos);
    CHECK(extract("Metadata/model_settings.config").find("name\" value=\"Second\"") != std::string::npos);
}
