/* Short-command wrapper for windres.
 *
 * QGroundControl's generated Windows resource script is compiled with the
 * target's full include and define list, which exceeds the Windows command
 * line limit once Qt and the CPM dependencies are in the tree.  Resource
 * metadata (icon/version) is not part of this custom build's acceptance, so
 * the wrapper compiles a fixed minimal version resource to the requested
 * output object instead of forwarding the oversized argument list.
 *
 * Build (MinGW):
 *   gcc -O2 -o rcwrap.exe tools/qgc_rc_wrapper.c
 * Use:
 *   cmake -B <build> -DCMAKE_RC_COMPILER=<abs path to rcwrap.exe>
 */
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <wchar.h>

static const char minimal_rc[] =
    "#include <windows.h>\n"
    "1 VERSIONINFO\n"
    "FILEVERSION 1,0,0,0\n"
    "PRODUCTVERSION 1,0,0,0\n"
    "FILEFLAGSMASK 0x3fL\n"
    "FILEFLAGS 0x0L\n"
    "FILEOS 0x40004L\n"
    "FILETYPE 0x1L\n"
    "FILESUBTYPE 0x0L\n"
    "BEGIN\n"
    "  BLOCK \"StringFileInfo\"\n"
    "  BEGIN\n"
    "    BLOCK \"040904b0\"\n"
    "    BEGIN\n"
    "      VALUE \"FileDescription\", \"Flight QGroundControl\"\n"
    "      VALUE \"ProductName\", \"Flight QGroundControl\"\n"
    "    END\n"
    "  END\n"
    "  BLOCK \"VarFileInfo\"\n"
    "  BEGIN\n"
    "    VALUE \"Translation\", 0x409, 1200\n"
    "  END\n"
    "END\n";

static int run(const wchar_t *command)
{
    STARTUPINFOW startup = {0};
    PROCESS_INFORMATION process = {0};
    startup.cb = sizeof(startup);
    if (!CreateProcessW(NULL, (LPWSTR)command, NULL, NULL, TRUE, 0, NULL, NULL,
                        &startup, &process))
        return 1;
    WaitForSingleObject(process.hProcess, INFINITE);
    DWORD code = 1;
    GetExitCodeProcess(process.hProcess, &code);
    CloseHandle(process.hThread);
    CloseHandle(process.hProcess);
    return (int)code;
}

int main(void)
{
    int argc = 0;
    wchar_t **argv = CommandLineToArgvW(GetCommandLineW(), &argc);
    if (!argv)
        return 1;
    const wchar_t *output = NULL;
    const wchar_t *depfile = NULL;
    const wchar_t *input = NULL;
    for (int i = 1; i < argc; ++i) {
        if ((!wcsicmp(argv[i], L"-o") || !wcsicmp(argv[i], L"--output") ||
             !wcsicmp(argv[i], L"/fo") || !wcsicmp(argv[i], L"/fo:")) && i + 1 < argc)
            output = argv[++i];
        else if (!wcsicmp(argv[i], L"-MF") && i + 1 < argc)
            depfile = argv[++i];
        else if (!wcsicmp(argv[i], L"/c") && i + 1 < argc)
            input = argv[++i];
        else {
            const size_t length = wcslen(argv[i]);
            if (length > 3 && !wcsicmp(argv[i] + length - 3, L".rc"))
                input = argv[i];
        }
    }
    if (!output)
        output = argv[argc - 1];
    if (!input || !output) {
        fwprintf(stderr, L"rcwrap: missing resource input or output\n");
        return 1;
    }

    wchar_t temp_dir[MAX_PATH], temp_rc[MAX_PATH];
    if (!GetTempPathW(MAX_PATH, temp_dir) ||
        !GetTempFileNameW(temp_dir, L"qgc", 0, temp_rc))
        return 1;
    FILE *file = _wfopen(temp_rc, L"wb");
    if (!file)
        return 1;
    fwrite(minimal_rc, 1, sizeof(minimal_rc) - 1, file);
    fclose(file);

    wchar_t command[4096];
    _snwprintf(command, 4096, L"windres -O coff \"%s\" -o \"%s\"", temp_rc, output);
    const int code = run(command);
    _wremove(temp_rc);

    if (code == 0 && depfile) {
        FILE *dep = _wfopen(depfile, L"wb");
        if (dep) {
            fwprintf(dep, L"%s: %s\n", output, argv[1]);
            fclose(dep);
        }
    }
    LocalFree(argv);
    return code;
}
