# PlatformIO pre-build script.
#
# build_flags go to both C and C++ compiles, so C++-only options in
# build_flags_reduce make the C compiler print a warning for every .c file:
#   cc1.exe: warning: command-line option '-fno-rtti' is valid for C++/D/ObjC++ but not for C
# Move those options out of build_flags and into CXXFLAGS, so C++ files still
# get them (including libraries and the framework) and C files do not.
# Environments that never had them in build_flags are left unchanged.

Import("env")

CXX_ONLY = ("-fno-rtti", "-fno-threadsafe-statics")

kept, moved = [], []
for line in env.get("BUILD_FLAGS", []):
    parts = str(line).split()
    hits = [p for p in parts if p in CXX_ONLY]
    if not hits:
        kept.append(line)
        continue
    moved.extend(hits)
    rest = " ".join(p for p in parts if p not in CXX_ONLY)
    if rest:
        kept.append(rest)

if moved:
    env.Replace(BUILD_FLAGS=kept)
    env.Append(CXXFLAGS=sorted(set(moved)))
