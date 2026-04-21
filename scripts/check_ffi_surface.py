#!/usr/bin/env python3
# Verify cffi decls and Python callsites agree on the FFI surface.
#
# Three bug classes this catches:
#   - declared-but-unresolved: defs_* names a function the compiled .so
#     does not export (missing SOURCE_FILES entry, missing __visible, or
#     a stale declaration). cffi raises AttributeError on first access.
#   - called-but-undeclared: klippy/*.py references ffi_lib.foo but no
#     defs_* block declares foo. Same AttributeError path.
#   - declared-but-wrong-signature: not caught here; surfaces as TypeError
#     at the call site and needs the Step 4 regression harness.
#
# Run from the repo root under WSL2. Exit 0 on match; non-zero and
# prints the offending names otherwise.
import pathlib, re, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "klippy"))
import chelper

ffi_main, ffi_lib = chelper.get_ffi()

# cffi's own parser is the source of truth for what was declared. Using
# it instead of a regex avoids false positives on parameter names,
# (void) lists, function-pointer types, typedefs, etc.
decl_names = {
    k.split(" ", 1)[1]
    for k in ffi_main._parser._declarations
    if k.startswith("function ")
}

unresolved = sorted(n for n in decl_names if not hasattr(ffi_lib, n))

call_pat = re.compile(r"ffi_lib\.([A-Za-z_][A-Za-z0-9_]*)")
caller_names = set()
for p in pathlib.Path("klippy").rglob("*.py"):
    try:
        caller_names.update(call_pat.findall(p.read_text(encoding="utf-8")))
    except UnicodeDecodeError:
        pass
uncalled = sorted(n for n in caller_names if not hasattr(ffi_lib, n))

if unresolved or uncalled:
    if unresolved:
        print("FAIL: declared but not in c_helper.so:")
        for n in unresolved:
            print(f"  - {n}")
    if uncalled:
        print("FAIL: called from Python but not declared / not resolved:")
        for n in uncalled:
            print(f"  - {n}")
    sys.exit(1)

print(f"OK: {len(decl_names)} declared, {len(caller_names)} called, all resolve.")
