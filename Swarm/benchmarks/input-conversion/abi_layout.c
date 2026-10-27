/* Build in an x64 MSVC developer shell: cl /nologo /W4 /WX /Iinclude
 * benchmarks\input-conversion\abi_layout.c /Fe:target\input-abi-layout.exe
 * Then: target\input-abi-layout.exe target\release\cpswcore.dll */
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include "swarm.h"

typedef int32_t (*GetAbi)(CSwarmAbiInfo *);
typedef SwarmInput *(*Prepare)(const CSwarmInput *);
typedef void (*FreeInput)(SwarmInput *);
int main(int argc, char **argv) {
    CSwarmAbiInfo abi = {0};
    GetAbi get_abi;
    Prepare prepare;
    FreeInput free_input;
    FARPROC proc;
    HMODULE dll;
    CInputPoint vertices[] = {{0,0}, {10,0}, {10,8}, {0,8}};
    float angles[] = {0};
    CInputItem item = {0};
    CSwarmInput input = {0};
    SwarmInput *prepared;
    if (argc != 2) return 2;
    dll = LoadLibraryA(argv[1]);
    if (!dll) return 3;
    proc = GetProcAddress(dll, "swarm_get_abi_info");
    if (!proc) return 4;
    memcpy(&get_abi, &proc, sizeof(get_abi));
    if (get_abi(&abi) != 0 || abi.abi_version != SWARM_ABI_VERSION ||
        abi.swarm_config_size != sizeof(CSwarmConfig)) {
        fprintf(stderr, "config ABI mismatch: header %u/%zu; DLL %u/%u\n",
            SWARM_ABI_VERSION, sizeof(CSwarmConfig), abi.abi_version, abi.swarm_config_size);
        return 5;
    }
    if (sizeof(CInputPoint) != 8 || sizeof(CInputHint) != 40 || sizeof(CInputItem) != 144 ||
        sizeof(CSwarmInput) != 104 || offsetof(CInputItem, hint) != 104 || offsetof(CSwarmInput, regions) != 72)
        return 6;
    proc = GetProcAddress(dll, "swarm_input_from_struct");
    if (!proc) return 7;
    memcpy(&prepare, &proc, sizeof(prepare));
    proc = GetProcAddress(dll, "swarm_input_free");
    if (!proc) return 8;
    memcpy(&free_input, &proc, sizeof(free_input));
    item.demand = 1; item.shape_kind = 1; item.vertices = vertices; item.vertex_count = 4;
    item.angles = angles; item.angle_count = 1; item.min_quality = -1;
    input.abi_version = SWARM_INPUT_ABI_VERSION; input.struct_size = (uint32_t)sizeof(input);
    input.items = &item; input.item_count = 1; input.strip_height = 50;
    input.group_workers = 1; input.quantum_ms = 250;
    prepared = prepare(&input);
    if (!prepared) return 9;
    free_input(prepared);
    FreeLibrary(dll);
    puts("C header ABI and direct input call passed.");
    return 0;
}
