#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <iomanip>

using namespace std;

// --- Constants ---
const double PI = 3.14159265358979323846;
const double INV_2POW32 = 1.0 / 4294967296.0;

// --- Bitwise & Hashing Helpers ---

inline uint32_t rotl32(uint32_t x, int8_t r) {
    return (x << r) | (x >> (32 - r));
}

inline uint32_t reverse_bits32(uint32_t x) {
    x = ((x >> 1) & 0x55555555) | ((x & 0x55555555) << 1);
    x = ((x >> 2) & 0x33333333) | ((x & 0x33333333) << 2);
    x = ((x >> 4) & 0x0F0F0F0F) | ((x & 0x0F0F0F0F) << 4);
    x = ((x >> 8) & 0x00FF00FF) | ((x & 0x00FF00FF) << 8);
    return (x >> 16) | (x << 16);
}

inline uint32_t fmix32(uint32_t h) {
    h ^= h >> 16;
    h *= 0x85ebca6b;
    h ^= h >> 13;
    h *= 0xc2b2ae35;
    h ^= h >> 16;
    return h;
}

inline uint32_t mix(uint32_t h, uint32_t k) {
    k *= 0xcc9e2d51;
    k = rotl32(k, 15);
    k *= 0x1b873593;
    h ^= k;
    h = rotl32(h, 13);
    h = h * 5 + 0xe6546b64;
    return h;
}

uint32_t hash_combine(uint32_t v0, uint32_t v1, uint32_t v2) {
    uint32_t h = 0x9747b28c;
    h = mix(h, v0);
    h = mix(h, v1);
    h = mix(h, v2);
    return fmix32(h);
}

uint32_t hash_combine(uint32_t v0, uint32_t v1) {
    uint32_t h = 0x9747b28c;
    h = mix(h, v0);
    h = mix(h, v1);
    return fmix32(h);
}

inline uint32_t laine_karras_permutation(uint32_t x, uint32_t seed) {
    x += seed;
    x ^= x * 0x6c50b47c;
    x ^= x * 0xb82f1e52;
    x ^= x * 0xc7afe638;
    x ^= x * 0x8d22f6e6;
    return x;
}

inline uint32_t nested_uniform_scramble(uint32_t x, uint32_t seed) {
    x = reverse_bits32(x);
    x = laine_karras_permutation(x, seed);
    x = reverse_bits32(x);
    return x;
}

// --- Sobol Logic ---

uint32_t sobol_dirs_0[32] = {
    0x80000000, 0x40000000, 0x20000000, 0x10000000, 0x08000000, 0x04000000, 0x02000000, 0x01000000,
    0x00800000, 0x00400000, 0x00200000, 0x00100000, 0x00080000, 0x00040000, 0x00020000, 0x00010000,
    0x00008000, 0x00004000, 0x00002000, 0x00001000, 0x00000800, 0x00000400, 0x00000200, 0x00000100,
    0x00000080, 0x00000040, 0x00000020, 0x00000010, 0x00000008, 0x00000004, 0x00000002, 0x00000001
};

uint32_t sobol_dirs_1[32] = {
    0x80000000, 0xc0000000, 0xc0000000, 0x90000000, 0xb8000000, 0xe8000000, 0xe2000000, 0xa3000000,
    0x8b000000, 0xce400000, 0xcee00000, 0x9aa00000, 0xb0080000, 0xe40c0000, 0xee0c0000, 0xaa090000,
    0x808b8000, 0xc0ce8000, 0xc0ce2000, 0x909a3000, 0xb8b0b000, 0xe8e4e400, 0xe2eeee00, 0xa3aaaa00,
    0x8b800080, 0xce8000c0, 0xce2000c0, 0x9a300090, 0xb0b000b8, 0xe4e400e8, 0xeeee00e2, 0xaaaa00a3
};

inline uint32_t sobol_u32_stateless(uint32_t index, uint32_t* dirs) {
    uint32_t g = index ^ (index >> 1);
    uint32_t x = 0;
    for (int bit = 0; g != 0 && bit < 32; ++bit) {
        if (g & 1) x ^= dirs[bit];
        g >>= 1;
    }
    return x;
}

// --- GGX VNDF Math ---

struct Vector3 {
    double x, y, z;
};

inline Vector3 normalize(Vector3 v) {
    double len = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len == 0.0) return {0, 0, 1};
    return {v.x / len, v.y / len, v.z / len};
}

Vector3 sample_ggx_vndf(double u1, double u2, Vector3 V, double roughness) {
    double alpha = roughness * roughness;
    
    // Warp and normalize view
    Vector3 Vh = normalize({alpha * V.x, alpha * V.y, V.z});

    // Orthonormal basis
    double lensq = Vh.x * Vh.x + Vh.y * Vh.y;
    Vector3 T1, T2;
    if (lensq > 1e-20) {
        double inv_len = 1.0 / sqrt(lensq);
        T1 = {-Vh.y * inv_len, Vh.x * inv_len, 0.0};
        // T2 = cross(Vh, T1)
        T2 = {Vh.y * T1.z - Vh.z * T1.y, Vh.z * T1.x - Vh.x * T1.z, Vh.x * T1.y - Vh.y * T1.x};
    } else {
        T1 = {1.0, 0.0, 0.0};
        T2 = {0.0, 1.0, 0.0};
    }

    // Parameterization of projected area
    double r = sqrt(u1);
    double phi = 2.0 * PI * u2;
    double t1 = r * cos(phi);
    double t2 = r * sin(phi);

    double s = 0.5 * (1.0 + Vh.z);
    t2 = (1.0 - s) * sqrt(max(0.0, 1.0 - t1 * t1)) + s * t2;

    // Reprojection onto hemisphere
    double t3 = sqrt(max(0.0, 1.0 - t1 * t1 - t2 * t2));
    Vector3 Nh = {
        t1 * T1.x + t2 * T2.x + t3 * Vh.x,
        t1 * T1.y + t2 * T2.y + t3 * Vh.y,
        t1 * T1.z + t2 * T2.z + t3 * Vh.z
    };

    // Transform back to ellipsoid
    Vector3 N = normalize({alpha * Nh.x, alpha * Nh.y, max(0.0, Nh.z)});
    return N;
}

int main() {
    const uint32_t N_SAMPLES = 10000000; // 10 Million
    const uint32_t pixel_id = 123;
    const uint32_t frame = 0;
    const uint32_t bounce = 0;
    const double roughness = 0.5;
    Vector3 V = normalize({0.2, -0.3, 0.93});

    uint32_t seed_base = hash_combine(pixel_id, frame, bounce);
    uint32_t scr_seed0 = hash_combine(seed_base, 0);
    uint32_t scr_seed1 = hash_combine(seed_base, 1);

    cout << "Starting CPU Benchmark: " << N_SAMPLES << " samples..." << endl;

    auto t_start = chrono::high_resolution_clock::now();

    double checksum = 0.0;
    for (uint32_t i = 0; i < N_SAMPLES; ++i) {
        // Full pipeline
        uint32_t idx = nested_uniform_scramble(i, seed_base);
        uint32_t x0 = sobol_u32_stateless(idx, sobol_dirs_0);
        uint32_t x1 = sobol_u32_stateless(idx, sobol_dirs_1);

        uint32_t u0_u32 = nested_uniform_scramble(x0, scr_seed0);
        uint32_t u1_u32 = nested_uniform_scramble(x1, scr_seed1);

        double u1 = u0_u32 * INV_2POW32;
        double u2 = u1_u32 * INV_2POW32;

        Vector3 n = sample_ggx_vndf(u1, u2, V, roughness);
        checksum += n.z; // Sum Z to ensure work isn't optimized away
    }

    auto t_end = chrono::high_resolution_clock::now();
    chrono::duration<double> diff = t_end - t_start;

    double samples_per_sec = N_SAMPLES / diff.count();
    
    cout << fixed << setprecision(3);
    cout << "-------------------------------------------" << endl;
    cout << "BENCHMARK_RESULTS (CPU_C++)" << endl;
    cout << "Total Time:     " << diff.count() << " s" << endl;
    cout << "Throughput:     " << (samples_per_sec / 1e6) << " M samples/sec" << endl;
    cout << "Latency:        " << (diff.count() * 1e9 / N_SAMPLES) << " ns/sample" << endl;
    cout << "Checksum (Z):   " << checksum << endl;
    cout << "-------------------------------------------" << endl;

    return 0;
}
