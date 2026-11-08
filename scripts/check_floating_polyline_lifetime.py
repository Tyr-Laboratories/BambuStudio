#!/usr/bin/env python3
"""Check production floating-polyline conversion and borrowed-grid lifetimes.

Uses an existing Ninja native test build's compiler/linker configuration. The
standalone probe includes the production translation unit to instrument private
functions too, with a sanitizer-aware wrapper at the borrowed-grid query.
It does not change the application or its build objects.
"""
import argparse
from pathlib import Path
import shlex
import subprocess


def strip_output_options(args):
    result = []
    index = 0
    while index < len(args):
        if args[index] in ("-o", "-c", "-MF", "-MT"):
            index += 2
        elif args[index] in ("-MD", "-Winvalid-pch"):
            index += 1
        elif (args[index:index + 2] in (["-Xclang", "-include-pch"], ["-Xclang", "-include"])
              and index + 3 < len(args) and "cmake_pch" in args[index + 3]):
            # Sanitizer compilation cannot reuse the uninstrumented PCH.
            index += 4
        else:
            result.append(args[index])
            index += 1
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build, output = args.build.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    commands = subprocess.check_output(
        ["ninja", "-C", str(build), "-t", "commands", "slicing_spatial_tests"], text=True).splitlines()
    compile_command = next(shlex.split(command) for command in commands
                           if " -c " in command and command.endswith("/FillFloatingConcentric.cpp"))
    obj, executable = output / "floating-lifetime.o", output / "floating-lifetime"
    compile_command = strip_output_options(compile_command)
    compile_command += ["-O1", "-fsanitize=address", "-fno-omit-frame-pointer", "-c",
                        str(root / "tests/fff_print/floating_polyline_lifetime_probe.cpp"), "-o", str(obj)]
    subprocess.run(compile_command, cwd=build, check=True)
    link_command = shlex.split(commands[-1])
    if link_command[:2] != [":", "&&"] or link_command[-2:] != ["&&", ":"]:
        raise RuntimeError("Unrecognized native test link command")
    link_command = strip_output_options(link_command[2:-2])
    link_command = [arg for arg in link_command if not arg.endswith(".cpp.o")]
    link_command += ["-fsanitize=address", str(obj), "-o", str(executable)]
    subprocess.run(link_command, cwd=build, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
