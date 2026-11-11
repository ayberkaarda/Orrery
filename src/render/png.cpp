#include "orrery/render/png.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <queue>
#include <string_view>
#include <system_error>
#include <utility>

namespace orrery::render {

namespace {

constexpr std::array<std::uint8_t, 8> kSignature{0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};
constexpr std::size_t kMaxStoredBlock = 65535;
constexpr std::size_t kMaxIdatChunk = 1U << 20U;
// Largest accepted image: 2^28 pixels (1 GiB of RGBA8). PNG itself allows
// 2^31 - 1 per dimension; this bound keeps sizes far from overflow. The
// inflater never produces more than the (4 * width + 1) * height scanline
// bytes the header announces, so it also bounds decompression.
constexpr std::uint64_t kMaxPixels = 1ULL << 28U;
constexpr std::uint32_t kMaxDimension = 0x7FFFFFFFU;
// Deflate expands at most 1032:1 (258-byte copies in 2 bits), so a stream of
// n bytes cannot need more output space than this many times n.
constexpr std::size_t kMaxInflateRatio = 1032;

constexpr std::array<std::uint32_t, 256> make_crc_table() {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t n = 0; n < 256; ++n) {
        std::uint32_t c = n;
        for (int k = 0; k < 8; ++k) {
            c = (c & 1U) != 0 ? 0xEDB88320U ^ (c >> 1U) : c >> 1U;
        }
        table[n] = c;
    }
    return table;
}

constexpr std::array<std::uint32_t, 256> kCrcTable = make_crc_table();

std::uint32_t crc32_update(std::uint32_t crc, std::span<const std::uint8_t> bytes) {
    for (const std::uint8_t byte : bytes) {
        crc = kCrcTable[(crc ^ byte) & 0xFFU] ^ (crc >> 8U);
    }
    return crc;
}

void put_u32_be(std::vector<std::uint8_t>& out, std::uint32_t value) {
    out.push_back(static_cast<std::uint8_t>(value >> 24U));
    out.push_back(static_cast<std::uint8_t>(value >> 16U));
    out.push_back(static_cast<std::uint8_t>(value >> 8U));
    out.push_back(static_cast<std::uint8_t>(value));
}

std::uint32_t get_u32_be(std::span<const std::uint8_t> bytes, std::size_t at) {
    return (static_cast<std::uint32_t>(bytes[at]) << 24U) |
           (static_cast<std::uint32_t>(bytes[at + 1]) << 16U) |
           (static_cast<std::uint32_t>(bytes[at + 2]) << 8U) |
           static_cast<std::uint32_t>(bytes[at + 3]);
}

void put_chunk(std::vector<std::uint8_t>& out, std::string_view type,
               std::span<const std::uint8_t> data) {
    put_u32_be(out, static_cast<std::uint32_t>(data.size()));
    const std::size_t type_at = out.size();
    out.insert(out.end(), type.begin(), type.end());
    out.insert(out.end(), data.begin(), data.end());
    const std::span<const std::uint8_t> crc_range(out.data() + type_at, 4 + data.size());
    put_u32_be(out, crc32(crc_range));
}

std::uint8_t paeth(std::uint8_t a, std::uint8_t b, std::uint8_t c) {
    const int p = int{a} + int{b} - int{c};
    const int pa = std::abs(p - int{a});
    const int pb = std::abs(p - int{b});
    const int pc = std::abs(p - int{c});
    if (pa <= pb && pa <= pc) {
        return a;
    }
    return pb <= pc ? b : c;
}

// Predictor of PNG filter `type` (1..4) from the left (a), upper (b) and
// upper-left (c) bytes; 0 for type 0.
std::uint8_t predictor(std::uint8_t type, std::uint8_t a, std::uint8_t b, std::uint8_t c) {
    switch (type) {
        case 1:
            return a;
        case 2:
            return b;
        case 3:
            return static_cast<std::uint8_t>((int{a} + int{b}) / 2);
        case 4:
            return paeth(a, b, c);
        default:
            return 0;
    }
}

using Failure = core::Failure<std::string>;

Failure fail(std::string message) {
    return core::failure(std::string("PNG: ") + std::move(message));
}

// ---------------------------------------------------------------------------
// Deflate (RFC 1951) tables.

constexpr std::size_t kLiteralCodes = 286; // 0..255 literals, 256 end, 257..285 lengths
constexpr std::size_t kDistanceCodes = 30;
constexpr std::size_t kCodeLengthCodes = 19;
constexpr unsigned kMaxCodeBits = 15;
constexpr unsigned kMaxCodeLengthBits = 7;
constexpr std::uint16_t kEndOfBlock = 256;
constexpr std::size_t kMinMatch = 3;
constexpr std::size_t kMaxMatch = 258;
constexpr std::size_t kWindow = 32768;

constexpr std::array<std::uint16_t, 29> kLengthBase{3,  4,  5,  6,   7,   8,   9,   10,  11, 13,
                                                    15, 17, 19, 23,  27,  31,  35,  43,  51, 59,
                                                    67, 83, 99, 115, 131, 163, 195, 227, 258};
constexpr std::array<std::uint8_t, 29> kLengthExtra{0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2,
                                                    2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
constexpr std::array<std::uint16_t, 30> kDistanceBase{
    1,   2,   3,   4,   5,   7,    9,    13,   17,   25,   33,   49,   65,    97,    129,
    193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
constexpr std::array<std::uint8_t, 30> kDistanceExtra{0, 0, 0,  0,  1,  1,  2,  2,  3,  3,
                                                      4, 4, 5,  5,  6,  6,  7,  7,  8,  8,
                                                      9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
constexpr std::array<std::uint8_t, kCodeLengthCodes> kCodeLengthOrder{
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

// Code lengths of the fixed Huffman codes (RFC 1951, 3.2.6).
std::array<std::uint8_t, 288> fixed_literal_lengths() {
    std::array<std::uint8_t, 288> lengths{};
    for (std::size_t i = 0; i < lengths.size(); ++i) {
        lengths[i] = static_cast<std::uint8_t>(i < 144 ? 8 : (i < 256 ? 9 : (i < 280 ? 7 : 8)));
    }
    return lengths;
}

std::size_t length_code(std::size_t length) {
    std::size_t code = 0;
    while (code + 1 < kLengthBase.size() && kLengthBase[code + 1] <= length) {
        ++code;
    }
    return code;
}

std::size_t distance_code(std::size_t distance) {
    std::size_t code = 0;
    while (code + 1 < kDistanceBase.size() && kDistanceBase[code + 1] <= distance) {
        ++code;
    }
    return code;
}

// ---------------------------------------------------------------------------
// Encoder.

class BitWriter {
public:
    explicit BitWriter(std::vector<std::uint8_t>& out) : out_(out) {}

    // Appends the low `bits` bits of `value`, least significant first.
    void put(std::uint32_t value, unsigned bits) {
        pending_ |= std::uint64_t{value & ((1U << bits) - 1U)} << count_;
        count_ += bits;
        while (count_ >= 8) {
            out_.push_back(static_cast<std::uint8_t>(pending_));
            pending_ >>= 8U;
            count_ -= 8;
        }
    }
    void align() {
        if (count_ > 0) {
            out_.push_back(static_cast<std::uint8_t>(pending_));
            pending_ = 0;
            count_ = 0;
        }
    }
    [[nodiscard]] std::vector<std::uint8_t>& bytes() { return out_; }

private:
    std::vector<std::uint8_t>& out_;
    std::uint64_t pending_ = 0;
    unsigned count_ = 0;
};

// Huffman code lengths for the given symbol frequencies, at most `max_bits`
// long. Symbols with frequency 0 get no code; at least two symbols get one,
// so that the code is complete. When the optimal tree is too deep, the
// frequencies are flattened (halved, kept nonzero) until it fits.
std::vector<std::uint8_t> huffman_lengths(std::vector<std::uint32_t> frequency, unsigned max_bits) {
    const std::size_t n = frequency.size();
    std::size_t used = 0;
    for (const std::uint32_t f : frequency) {
        used += f > 0 ? 1U : 0U;
    }
    for (std::size_t i = 0; i < n && used < 2; ++i) {
        if (frequency[i] == 0) {
            frequency[i] = 1;
            ++used;
        }
    }
    std::vector<std::uint8_t> lengths(n, 0);
    for (;;) {
        // Nodes 0..n-1 are leaves; internal nodes follow. Ties are broken
        // by node index, so the result is deterministic.
        struct Node {
            std::uint64_t weight;
            std::size_t index;
        };
        const auto heavier = [](const Node& a, const Node& b) {
            return a.weight != b.weight ? a.weight > b.weight : a.index > b.index;
        };
        std::priority_queue<Node, std::vector<Node>, decltype(heavier)> queue(heavier);
        std::vector<std::size_t> parent(n, 0);
        for (std::size_t i = 0; i < n; ++i) {
            if (frequency[i] > 0) {
                queue.push({frequency[i], i});
            }
        }
        std::size_t next = n;
        while (queue.size() > 1) {
            const Node a = queue.top();
            queue.pop();
            const Node b = queue.top();
            queue.pop();
            parent.push_back(0);
            parent[a.index] = next;
            parent[b.index] = next;
            queue.push({a.weight + b.weight, next});
            ++next;
        }
        const std::size_t root = next - 1;
        // Depths: parents always have larger indices than their children.
        std::vector<std::uint8_t> depth(parent.size(), 0);
        unsigned deepest = 0;
        for (std::size_t i = parent.size(); i-- > 0;) {
            if (i != root && (i >= n || frequency[i] > 0)) {
                depth[i] =
                    static_cast<std::uint8_t>(std::min<unsigned>(depth[parent[i]] + 1U, 255U));
                deepest = std::max<unsigned>(deepest, depth[i]);
            }
        }
        if (deepest <= max_bits) {
            for (std::size_t i = 0; i < n; ++i) {
                lengths[i] = frequency[i] > 0 ? depth[i] : 0;
            }
            return lengths;
        }
        for (std::uint32_t& f : frequency) {
            if (f > 0) {
                f = (f >> 1U) | 1U;
            }
        }
    }
}

// Canonical codes (RFC 1951, 3.2.2), bit-reversed for least-significant-
// first output.
std::vector<std::uint16_t> canonical_codes(const std::vector<std::uint8_t>& lengths) {
    std::array<std::uint16_t, kMaxCodeBits + 1> count{};
    for (const std::uint8_t length : lengths) {
        ++count[length];
    }
    count[0] = 0;
    std::array<std::uint16_t, kMaxCodeBits + 2> next{};
    std::uint32_t code = 0;
    for (std::size_t bits = 1; bits <= kMaxCodeBits; ++bits) {
        code = (code + count[bits - 1]) << 1U;
        next[bits] = static_cast<std::uint16_t>(code);
    }
    std::vector<std::uint16_t> codes(lengths.size(), 0);
    for (std::size_t i = 0; i < lengths.size(); ++i) {
        const unsigned length = lengths[i];
        if (length == 0) {
            continue;
        }
        std::uint32_t value = next[length]++;
        std::uint32_t reversed = 0;
        for (unsigned b = 0; b < length; ++b) {
            reversed = (reversed << 1U) | (value & 1U);
            value >>= 1U;
        }
        codes[i] = static_cast<std::uint16_t>(reversed);
    }
    return codes;
}

// One LZ77 token: a literal byte (distance 0) or a copy.
struct Token {
    std::uint16_t value;    // literal byte, or copy length 3..258
    std::uint16_t distance; // 0 for a literal, else 1..32768
};

struct Match {
    std::size_t length = 0;
    std::size_t distance = 0;
};

// LZ77 with hash chains over 3-byte prefixes, a 32 KiB window and one step
// of lazy matching (a match is deferred when the next position has a longer
// one).
class Matcher {
public:
    explicit Matcher(std::span<const std::uint8_t> data)
        : data_(data), head_(kHashSize, kNone), previous_(kWindow, kNone) {}

    std::vector<Token> tokens() {
        std::vector<Token> out;
        out.reserve(data_.size() / 4 + 16);
        std::size_t i = 0;
        std::optional<Match> pending;
        while (i < data_.size()) {
            const Match match = pending ? *pending : find(i);
            pending.reset();
            insert(i);
            if (match.length >= kMinMatch) {
                if (match.length < kLazyLimit && i + 1 < data_.size()) {
                    const Match later = find(i + 1);
                    if (later.length > match.length) {
                        out.push_back({data_[i], 0});
                        pending = later;
                        ++i;
                        continue;
                    }
                }
                out.push_back({static_cast<std::uint16_t>(match.length),
                               static_cast<std::uint16_t>(match.distance)});
                for (std::size_t k = 1; k < match.length; ++k) {
                    insert(i + k);
                }
                i += match.length;
            } else {
                out.push_back({data_[i], 0});
                ++i;
            }
        }
        return out;
    }

private:
    static constexpr std::size_t kHashBits = 15;
    static constexpr std::size_t kHashSize = std::size_t{1} << kHashBits;
    static constexpr std::size_t kMaxChain = 96;
    static constexpr std::size_t kLazyLimit = 32;
    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

    [[nodiscard]] std::size_t hash(std::size_t i) const {
        const std::uint32_t key =
            (std::uint32_t{data_[i]} << 16U) | (std::uint32_t{data_[i + 1]} << 8U) | data_[i + 2];
        return (key * 2654435761U) >> (32U - kHashBits);
    }

    void insert(std::size_t i) {
        if (i + kMinMatch > data_.size()) {
            return;
        }
        const std::size_t h = hash(i);
        previous_[i % kWindow] = head_[h];
        head_[h] = i;
    }

    [[nodiscard]] Match find(std::size_t i) const {
        Match best;
        if (i + kMinMatch > data_.size()) {
            return best;
        }
        const std::size_t limit = std::min(kMaxMatch, data_.size() - i);
        std::size_t candidate = head_[hash(i)];
        for (std::size_t chain = 0;
             chain < kMaxChain && candidate != kNone && candidate < i && i - candidate <= kWindow;
             ++chain) {
            if (data_[candidate + best.length] == data_[i + best.length] || best.length == 0) {
                std::size_t length = 0;
                while (length < limit && data_[candidate + length] == data_[i + length]) {
                    ++length;
                }
                if (length > best.length) {
                    best = {length, i - candidate};
                    if (length == limit) {
                        break;
                    }
                }
            }
            const std::size_t older = previous_[candidate % kWindow];
            if (older == kNone || older >= candidate) {
                break; // the slot was reused by a newer position
            }
            candidate = older;
        }
        if (best.length < kMinMatch) {
            return {};
        }
        return best;
    }

    std::span<const std::uint8_t> data_;
    std::vector<std::size_t> head_;
    std::vector<std::size_t> previous_;
};

struct CodeLengthSymbol {
    std::uint8_t symbol; // 0..18
    std::uint8_t extra;  // repeat count minus its base for 16, 17, 18
};

// Run-length encoding of code lengths with the symbols 16, 17 and 18.
std::vector<CodeLengthSymbol> run_lengths(const std::vector<std::uint8_t>& lengths) {
    std::vector<CodeLengthSymbol> out;
    std::size_t i = 0;
    while (i < lengths.size()) {
        const std::uint8_t value = lengths[i];
        std::size_t run = 1;
        while (i + run < lengths.size() && lengths[i + run] == value) {
            ++run;
        }
        i += run;
        if (value == 0) {
            while (run >= 11) {
                const std::size_t k = std::min<std::size_t>(run, 138);
                out.push_back({18, static_cast<std::uint8_t>(k - 11)});
                run -= k;
            }
            if (run >= 3) {
                out.push_back({17, static_cast<std::uint8_t>(run - 3)});
                run = 0;
            }
        } else {
            out.push_back({value, 0});
            --run;
            while (run >= 3) {
                const std::size_t k = std::min<std::size_t>(run, 6);
                out.push_back({16, static_cast<std::uint8_t>(k - 3)});
                run -= k;
            }
        }
        for (; run > 0; --run) {
            out.push_back({value, 0});
        }
    }
    return out;
}

unsigned code_length_extra_bits(std::uint8_t symbol) {
    return symbol == 16 ? 2U : (symbol == 17 ? 3U : (symbol == 18 ? 7U : 0U));
}

// Writes one block of tokens covering data[begin, end) as a stored, fixed
// or dynamic Huffman block, whichever is smallest.
void write_block(BitWriter& out, std::span<const Token> tokens, std::span<const std::uint8_t> data,
                 std::size_t begin, std::size_t end, bool final_block) {
    std::vector<std::uint32_t> literal_frequency(kLiteralCodes, 0);
    std::vector<std::uint32_t> distance_frequency(kDistanceCodes, 0);
    std::uint64_t extra_bits = 0;
    for (const Token& t : tokens) {
        if (t.distance == 0) {
            ++literal_frequency[t.value];
        } else {
            const std::size_t lc = length_code(t.value);
            const std::size_t dc = distance_code(t.distance);
            ++literal_frequency[257 + lc];
            ++distance_frequency[dc];
            extra_bits += std::uint64_t{kLengthExtra[lc]} + kDistanceExtra[dc];
        }
    }
    literal_frequency[kEndOfBlock] = 1;

    // Fixed Huffman cost.
    const std::array<std::uint8_t, 288> fixed = fixed_literal_lengths();
    std::uint64_t fixed_bits = 3 + extra_bits;
    for (std::size_t s = 0; s < kLiteralCodes; ++s) {
        fixed_bits += std::uint64_t{literal_frequency[s]} * fixed[s];
    }
    for (std::size_t s = 0; s < kDistanceCodes; ++s) {
        fixed_bits += std::uint64_t{distance_frequency[s]} * 5U;
    }

    // Dynamic Huffman cost, header included.
    const std::vector<std::uint8_t> literal_lengths =
        huffman_lengths(literal_frequency, kMaxCodeBits);
    const std::vector<std::uint8_t> distance_lengths =
        huffman_lengths(distance_frequency, kMaxCodeBits);
    std::size_t hlit = kLiteralCodes;
    while (hlit > 257 && literal_lengths[hlit - 1] == 0) {
        --hlit;
    }
    std::size_t hdist = kDistanceCodes;
    while (hdist > 1 && distance_lengths[hdist - 1] == 0) {
        --hdist;
    }
    std::vector<std::uint8_t> all_lengths(
        literal_lengths.begin(), literal_lengths.begin() + static_cast<std::ptrdiff_t>(hlit));
    all_lengths.insert(all_lengths.end(), distance_lengths.begin(),
                       distance_lengths.begin() + static_cast<std::ptrdiff_t>(hdist));
    const std::vector<CodeLengthSymbol> rle = run_lengths(all_lengths);
    std::vector<std::uint32_t> cl_frequency(kCodeLengthCodes, 0);
    for (const CodeLengthSymbol& c : rle) {
        ++cl_frequency[c.symbol];
    }
    const std::vector<std::uint8_t> cl_lengths = huffman_lengths(cl_frequency, kMaxCodeLengthBits);
    std::size_t hclen = kCodeLengthCodes;
    while (hclen > 4 && cl_lengths[kCodeLengthOrder[hclen - 1]] == 0) {
        --hclen;
    }
    std::uint64_t dynamic_bits = 3 + 5 + 5 + 4 + 3 * hclen + extra_bits;
    for (const CodeLengthSymbol& c : rle) {
        dynamic_bits += cl_lengths[c.symbol] + code_length_extra_bits(c.symbol);
    }
    for (std::size_t s = 0; s < kLiteralCodes; ++s) {
        dynamic_bits += std::uint64_t{literal_frequency[s]} * literal_lengths[s];
    }
    for (std::size_t s = 0; s < kDistanceCodes; ++s) {
        dynamic_bits += std::uint64_t{distance_frequency[s]} * distance_lengths[s];
    }

    // Stored cost: header, padding (at most 7 bits) and LEN/NLEN per block.
    const std::size_t stored_bytes = end - begin;
    const std::size_t stored_blocks =
        std::max<std::size_t>(1, (stored_bytes + kMaxStoredBlock - 1) / kMaxStoredBlock);
    const std::uint64_t stored_bits =
        stored_blocks * (3 + 7 + 32) + std::uint64_t{stored_bytes} * 8;

    if (stored_bits < fixed_bits && stored_bits < dynamic_bits) {
        std::size_t at = begin;
        do {
            const std::size_t length = std::min(kMaxStoredBlock, end - at);
            const bool last = final_block && at + length == end;
            out.put(last ? 1U : 0U, 1);
            out.put(0, 2);
            out.align();
            out.put(static_cast<std::uint32_t>(length), 16);
            out.put(static_cast<std::uint32_t>(~length & 0xFFFFU), 16);
            std::vector<std::uint8_t>& bytes = out.bytes();
            bytes.insert(bytes.end(), data.begin() + static_cast<std::ptrdiff_t>(at),
                         data.begin() + static_cast<std::ptrdiff_t>(at + length));
            at += length;
        } while (at < end);
        return;
    }

    std::vector<std::uint8_t> lit_lengths;
    std::vector<std::uint8_t> dist_lengths;
    out.put(final_block ? 1U : 0U, 1);
    if (fixed_bits <= dynamic_bits) {
        out.put(1, 2);
        lit_lengths.assign(fixed.begin(), fixed.end());
        dist_lengths.assign(kDistanceCodes, 5);
    } else {
        out.put(2, 2);
        out.put(static_cast<std::uint32_t>(hlit - 257), 5);
        out.put(static_cast<std::uint32_t>(hdist - 1), 5);
        out.put(static_cast<std::uint32_t>(hclen - 4), 4);
        for (std::size_t k = 0; k < hclen; ++k) {
            out.put(cl_lengths[kCodeLengthOrder[k]], 3);
        }
        const std::vector<std::uint16_t> cl_codes = canonical_codes(cl_lengths);
        for (const CodeLengthSymbol& c : rle) {
            out.put(cl_codes[c.symbol], cl_lengths[c.symbol]);
            out.put(c.extra, code_length_extra_bits(c.symbol));
        }
        lit_lengths = literal_lengths;
        dist_lengths = distance_lengths;
    }
    const std::vector<std::uint16_t> lit_codes = canonical_codes(lit_lengths);
    const std::vector<std::uint16_t> dist_codes = canonical_codes(dist_lengths);
    for (const Token& t : tokens) {
        if (t.distance == 0) {
            out.put(lit_codes[t.value], lit_lengths[t.value]);
            continue;
        }
        const std::size_t lc = length_code(t.value);
        out.put(lit_codes[257 + lc], lit_lengths[257 + lc]);
        out.put(static_cast<std::uint32_t>(t.value - kLengthBase[lc]), kLengthExtra[lc]);
        const std::size_t dc = distance_code(t.distance);
        out.put(dist_codes[dc], dist_lengths[dc]);
        out.put(static_cast<std::uint32_t>(t.distance - kDistanceBase[dc]), kDistanceExtra[dc]);
    }
    out.put(lit_codes[kEndOfBlock], lit_lengths[kEndOfBlock]);
}

// zlib stream (RFC 1950) of `data`: header, deflate blocks, Adler-32.
std::vector<std::uint8_t> zlib_compress(std::span<const std::uint8_t> data) {
    std::vector<std::uint8_t> zlib;
    zlib.reserve(data.size() / 4 + 64);
    // CMF 0x78: deflate, 32 KiB window; FLG 0xDA: no dictionary, maximum
    // compression level, check bits so that (CMF * 256 + FLG) % 31 == 0.
    zlib.push_back(0x78);
    zlib.push_back(0xDA);
    const std::vector<Token> tokens = Matcher(data).tokens();
    BitWriter out(zlib);
    constexpr std::size_t kTokensPerBlock = 1U << 15U;
    std::size_t first = 0;
    std::size_t position = 0;
    do {
        const std::size_t last = std::min(tokens.size(), first + kTokensPerBlock);
        std::size_t end = position;
        for (std::size_t k = first; k < last; ++k) {
            end += tokens[k].distance == 0 ? 1U : tokens[k].value;
        }
        write_block(out, std::span<const Token>(tokens.data() + first, last - first), data,
                    position, end, last == tokens.size());
        first = last;
        position = end;
    } while (first < tokens.size());
    out.align();
    put_u32_be(zlib, adler32(data));
    return zlib;
}

// Filtered scanlines: per row the filter type (0..4) whose output has the
// smallest sum of absolute values as signed bytes (the usual heuristic for
// photographic content), followed by the filtered bytes.
std::vector<std::uint8_t> filter_scanlines(const ImageRgba8& image) {
    const std::size_t row_bytes = std::size_t{image.width} * 4U;
    std::vector<std::uint8_t> raw;
    raw.reserve((row_bytes + 1) * image.height);
    std::array<std::vector<std::uint8_t>, 5> candidates;
    for (std::vector<std::uint8_t>& c : candidates) {
        c.resize(row_bytes);
    }
    for (std::uint32_t y = 0; y < image.height; ++y) {
        const std::uint8_t* row = image.pixels.data() + image.offset(0, y);
        const std::uint8_t* above = y > 0 ? row - row_bytes : nullptr;
        std::uint64_t best_score = 0;
        std::uint8_t best = 0;
        for (std::uint8_t type = 0; type < 5; ++type) {
            std::vector<std::uint8_t>& out = candidates[type];
            std::uint64_t score = 0;
            for (std::size_t i = 0; i < row_bytes; ++i) {
                const std::uint8_t a = i >= 4 ? row[i - 4] : 0;
                const std::uint8_t b = above != nullptr ? above[i] : 0;
                const std::uint8_t c = above != nullptr && i >= 4 ? above[i - 4] : 0;
                out[i] = static_cast<std::uint8_t>(row[i] - predictor(type, a, b, c));
                score += std::min<unsigned>(out[i], 256U - out[i]);
            }
            if (type == 0 || score < best_score) {
                best_score = score;
                best = type;
            }
        }
        raw.push_back(best);
        raw.insert(raw.end(), candidates[best].begin(), candidates[best].end());
    }
    return raw;
}

// ---------------------------------------------------------------------------
// Inflater.

// Reads bits least significant first; reading past the end sets a flag and
// returns zeros, which the callers check after every symbol.
class BitReader {
public:
    explicit BitReader(std::span<const std::uint8_t> data) : data_(data) {}

    std::uint32_t bits(unsigned count) {
        std::uint32_t value = 0;
        for (unsigned i = 0; i < count; ++i) {
            value |= bit() << i;
        }
        return value;
    }
    std::uint32_t bit() {
        if (byte_ >= data_.size()) {
            overrun_ = true;
            return 0;
        }
        const std::uint32_t b = (data_[byte_] >> bit_) & 1U;
        if (++bit_ == 8) {
            bit_ = 0;
            ++byte_;
        }
        return b;
    }
    void align() {
        if (bit_ != 0) {
            bit_ = 0;
            ++byte_;
        }
    }
    [[nodiscard]] bool overrun() const noexcept { return overrun_; }
    // Whole bytes left (after align()).
    [[nodiscard]] std::size_t remaining() const noexcept {
        return byte_ < data_.size() ? data_.size() - byte_ : 0;
    }
    [[nodiscard]] std::size_t position() const noexcept { return byte_; }
    void skip(std::size_t bytes) { byte_ += bytes; }

private:
    std::span<const std::uint8_t> data_;
    std::size_t byte_ = 0;
    unsigned bit_ = 0;
    bool overrun_ = false;
};

// Canonical Huffman decoding table: number of codes per length and the
// symbols in code order (as in the zlib reference decoder "puff").
struct Decoder {
    std::array<std::uint16_t, kMaxCodeBits + 1> count{};
    std::vector<std::uint16_t> symbol;
};

enum class CodeShape : std::uint8_t { complete, single, empty, incomplete, over_subscribed };

CodeShape build_decoder(std::span<const std::uint8_t> lengths, Decoder& decoder) {
    decoder.count.fill(0);
    for (const std::uint8_t length : lengths) {
        ++decoder.count[length];
    }
    const std::size_t used = lengths.size() - decoder.count[0];
    int left = 1;
    for (std::size_t bits = 1; bits <= kMaxCodeBits; ++bits) {
        left = left * 2 - decoder.count[bits];
        if (left < 0) {
            return CodeShape::over_subscribed;
        }
    }
    std::array<std::uint16_t, kMaxCodeBits + 2> offset{};
    for (std::size_t bits = 1; bits <= kMaxCodeBits; ++bits) {
        offset[bits + 1] = static_cast<std::uint16_t>(offset[bits] + decoder.count[bits]);
    }
    decoder.symbol.assign(used, 0);
    for (std::size_t s = 0; s < lengths.size(); ++s) {
        if (lengths[s] != 0) {
            decoder.symbol[offset[lengths[s]]++] = static_cast<std::uint16_t>(s);
        }
    }
    if (left == 0) {
        return CodeShape::complete;
    }
    if (used == 0) {
        return CodeShape::empty;
    }
    return used == 1 && decoder.count[1] == 1 ? CodeShape::single : CodeShape::incomplete;
}

// Decodes one symbol; std::nullopt if the bits match no code.
std::optional<std::uint16_t> decode_symbol(BitReader& in, const Decoder& decoder) {
    int code = 0;
    int first = 0;
    int index = 0;
    for (std::size_t bits = 1; bits <= kMaxCodeBits; ++bits) {
        code |= static_cast<int>(in.bit());
        const int count = decoder.count[bits];
        if (code - count < first) {
            return decoder
                .symbol[static_cast<std::size_t>(index) + static_cast<std::size_t>(code - first)];
        }
        index += count;
        first = (first + count) << 1;
        code <<= 1;
    }
    return std::nullopt;
}

using InflateResult = core::Result<std::monostate, std::string>;

// Decodes the codes of one fixed or dynamic block into `out`.
InflateResult inflate_codes(BitReader& in, const Decoder& literals, const Decoder& distances,
                            std::vector<std::uint8_t>& out, std::size_t limit) {
    for (;;) {
        const std::optional<std::uint16_t> symbol = decode_symbol(in, literals);
        if (in.overrun()) {
            return fail("truncated deflate stream");
        }
        if (!symbol) {
            return fail("invalid literal/length code");
        }
        if (*symbol < 256) {
            if (out.size() >= limit) {
                return fail("deflate data exceeds the image size");
            }
            out.push_back(static_cast<std::uint8_t>(*symbol));
            continue;
        }
        if (*symbol == kEndOfBlock) {
            return std::monostate{};
        }
        const std::size_t lc = *symbol - 257U;
        if (lc >= kLengthBase.size()) {
            return fail("invalid literal/length symbol " + std::to_string(*symbol));
        }
        const std::size_t length = kLengthBase[lc] + in.bits(kLengthExtra[lc]);
        const std::optional<std::uint16_t> dc = decode_symbol(in, distances);
        if (in.overrun()) {
            return fail("truncated deflate stream");
        }
        if (!dc) {
            return fail("invalid distance code");
        }
        if (*dc >= kDistanceBase.size()) {
            return fail("invalid distance symbol " + std::to_string(*dc));
        }
        const std::size_t distance = kDistanceBase[*dc] + in.bits(kDistanceExtra[*dc]);
        if (in.overrun()) {
            return fail("truncated deflate stream");
        }
        if (distance > out.size()) {
            return fail("distance " + std::to_string(distance) +
                        " reaches before the start of the output");
        }
        if (length > limit - out.size()) {
            return fail("deflate data exceeds the image size");
        }
        const std::size_t from = out.size() - distance;
        for (std::size_t k = 0; k < length; ++k) {
            out.push_back(out[from + k]);
        }
    }
}

// Reads the code lengths of a dynamic block and builds its decoders.
InflateResult read_dynamic_codes(BitReader& in, Decoder& literals, Decoder& distances) {
    const std::size_t hlit = in.bits(5) + 257U;
    const std::size_t hdist = in.bits(5) + 1U;
    const std::size_t hclen = in.bits(4) + 4U;
    if (hlit > kLiteralCodes || hdist > kDistanceCodes) {
        return fail("dynamic block with too many length or distance codes");
    }
    std::array<std::uint8_t, kCodeLengthCodes> cl_lengths{};
    for (std::size_t k = 0; k < hclen; ++k) {
        cl_lengths[kCodeLengthOrder[k]] = static_cast<std::uint8_t>(in.bits(3));
    }
    if (in.overrun()) {
        return fail("truncated deflate stream");
    }
    Decoder cl;
    const CodeShape cl_shape = build_decoder(cl_lengths, cl);
    if (cl_shape == CodeShape::over_subscribed) {
        return fail("over-subscribed code-length code");
    }
    if (cl_shape != CodeShape::complete) {
        return fail("incomplete code-length code");
    }
    std::array<std::uint8_t, kLiteralCodes + kDistanceCodes> lengths{};
    const std::size_t total = hlit + hdist;
    std::size_t index = 0;
    while (index < total) {
        const std::optional<std::uint16_t> symbol = decode_symbol(in, cl);
        if (in.overrun()) {
            return fail("truncated deflate stream");
        }
        if (!symbol) {
            return fail("invalid code-length code");
        }
        if (*symbol < 16) {
            lengths[index++] = static_cast<std::uint8_t>(*symbol);
            continue;
        }
        std::uint8_t value = 0;
        std::size_t repeat = 0;
        if (*symbol == 16) {
            if (index == 0) {
                return fail("code-length repeat without a previous length");
            }
            value = lengths[index - 1];
            repeat = 3 + in.bits(2);
        } else if (*symbol == 17) {
            repeat = 3 + in.bits(3);
        } else {
            repeat = 11 + in.bits(7);
        }
        if (repeat > total - index) {
            return fail("code-length repeat beyond the declared code count");
        }
        for (std::size_t k = 0; k < repeat; ++k) {
            lengths[index++] = value;
        }
    }
    if (in.overrun()) {
        return fail("truncated deflate stream");
    }
    if (lengths[kEndOfBlock] == 0) {
        return fail("dynamic block without an end-of-block code");
    }
    const std::span<const std::uint8_t> literal_lengths(lengths.data(), hlit);
    const std::span<const std::uint8_t> distance_lengths(lengths.data() + hlit, hdist);
    const CodeShape lit_shape = build_decoder(literal_lengths, literals);
    if (lit_shape != CodeShape::complete && lit_shape != CodeShape::single) {
        return fail("invalid literal/length code lengths");
    }
    const CodeShape dist_shape = build_decoder(distance_lengths, distances);
    if (dist_shape == CodeShape::over_subscribed || dist_shape == CodeShape::incomplete) {
        return fail("invalid distance code lengths");
    }
    return std::monostate{};
}

// Inflates a zlib stream that must decompress to exactly `expected` bytes.
// Output beyond `expected` is an error as soon as it would be written, so a
// small stream can never make the reader allocate more than the image needs.
core::Result<std::vector<std::uint8_t>, std::string>
inflate_zlib(std::span<const std::uint8_t> zlib, std::size_t expected) {
    if (zlib.size() < 6) {
        return fail("zlib stream too short");
    }
    if ((zlib[0] & 0x0FU) != 8 || (zlib[0] >> 4U) > 7 || (zlib[0] * 256U + zlib[1]) % 31U != 0 ||
        (zlib[1] & 0x20U) != 0) {
        return fail("invalid zlib header");
    }
    std::vector<std::uint8_t> out;
    out.reserve(std::min(expected, (zlib.size() - 2) * kMaxInflateRatio));
    BitReader in(zlib.subspan(2));
    Decoder fixed_literals;
    Decoder fixed_distances;
    {
        const std::array<std::uint8_t, 288> lengths = fixed_literal_lengths();
        (void)build_decoder(lengths, fixed_literals);
        const std::array<std::uint8_t, 30> distance_lengths = [] {
            std::array<std::uint8_t, 30> d{};
            d.fill(5);
            return d;
        }();
        (void)build_decoder(distance_lengths, fixed_distances);
    }
    bool final_block = false;
    while (!final_block) {
        final_block = in.bit() != 0;
        const std::uint32_t type = in.bits(2);
        if (in.overrun()) {
            return fail("truncated deflate stream");
        }
        if (type == 0) {
            in.align();
            if (in.remaining() < 4) {
                return fail("truncated deflate stream");
            }
            const std::uint32_t length = in.bits(16);
            const std::uint32_t complement = in.bits(16);
            if ((length ^ 0xFFFFU) != complement) {
                return fail("stored block length check failed");
            }
            if (in.remaining() < length) {
                return fail("truncated deflate stream");
            }
            if (length > expected - out.size()) {
                return fail("deflate data exceeds the image size");
            }
            const std::size_t at = 2 + in.position();
            out.insert(out.end(), zlib.begin() + static_cast<std::ptrdiff_t>(at),
                       zlib.begin() + static_cast<std::ptrdiff_t>(at + length));
            in.skip(length);
        } else if (type == 1) {
            auto result = inflate_codes(in, fixed_literals, fixed_distances, out, expected);
            if (!result) {
                return core::failure(result.error());
            }
        } else if (type == 2) {
            Decoder literals;
            Decoder distances;
            auto codes = read_dynamic_codes(in, literals, distances);
            if (!codes) {
                return core::failure(codes.error());
            }
            auto result = inflate_codes(in, literals, distances, out, expected);
            if (!result) {
                return core::failure(result.error());
            }
        } else {
            return fail("invalid deflate block type 3 (reserved)");
        }
    }
    in.align();
    if (in.remaining() < 4) {
        return fail("truncated zlib stream: Adler-32 missing");
    }
    if (in.remaining() > 4) {
        return fail("unexpected data after the final deflate block");
    }
    if (adler32(out) != get_u32_be(zlib, 2 + in.position())) {
        return fail("Adler-32 mismatch");
    }
    if (out.size() != expected) {
        return fail("decompressed size " + std::to_string(out.size()) + " != expected " +
                    std::to_string(expected));
    }
    return out;
}

} // namespace

std::uint32_t crc32(std::span<const std::uint8_t> bytes) noexcept {
    return crc32_update(0xFFFFFFFFU, bytes) ^ 0xFFFFFFFFU;
}

std::uint32_t adler32(std::span<const std::uint8_t> bytes) noexcept {
    constexpr std::uint32_t kModulus = 65521;
    // 5552 bytes is the largest run whose sums cannot overflow 32 bits.
    constexpr std::size_t kRun = 5552;
    std::uint32_t a = 1;
    std::uint32_t b = 0;
    std::size_t at = 0;
    while (at < bytes.size()) {
        const std::size_t end = std::min(bytes.size(), at + kRun);
        for (; at < end; ++at) {
            a += bytes[at];
            b += a;
        }
        a %= kModulus;
        b %= kModulus;
    }
    return (b << 16U) | a;
}

core::Result<std::vector<std::uint8_t>, std::string> encode_png(const ImageRgba8& image) {
    if (!image.valid()) {
        return fail("image is empty or its pixel buffer does not match width * height * 4");
    }
    if (image.width > kMaxDimension || image.height > kMaxDimension ||
        std::uint64_t{image.width} * image.height > kMaxPixels) {
        return fail("image is too large to encode");
    }

    const std::vector<std::uint8_t> raw = filter_scanlines(image);
    const std::vector<std::uint8_t> zlib = zlib_compress(raw);

    std::vector<std::uint8_t> png(kSignature.begin(), kSignature.end());
    std::vector<std::uint8_t> header;
    put_u32_be(header, image.width);
    put_u32_be(header, image.height);
    header.insert(header.end(), {8, 6, 0, 0, 0}); // depth 8, RGBA, deflate, filter 0, no interlace
    put_chunk(png, "IHDR", header);
    for (std::size_t offset = 0; offset < zlib.size(); offset += kMaxIdatChunk) {
        const std::size_t length = std::min(kMaxIdatChunk, zlib.size() - offset);
        put_chunk(png, "IDAT", std::span<const std::uint8_t>(zlib.data() + offset, length));
    }
    put_chunk(png, "IEND", {});
    return png;
}

core::Result<ImageRgba8, std::string> decode_png(std::span<const std::uint8_t> bytes) {
    if (bytes.size() < kSignature.size() ||
        !std::equal(kSignature.begin(), kSignature.end(), bytes.begin())) {
        return fail("missing PNG signature");
    }

    ImageRgba8 image;
    bool have_header = false;
    bool have_end = false;
    std::vector<std::uint8_t> zlib;
    std::size_t at = kSignature.size();
    while (at < bytes.size()) {
        if (bytes.size() - at < 12) {
            return fail("truncated chunk at byte " + std::to_string(at));
        }
        const std::uint32_t length = get_u32_be(bytes, at);
        if (length > 0x7FFFFFFFU || bytes.size() - at - 12 < length) {
            return fail("chunk length " + std::to_string(length) + " at byte " +
                        std::to_string(at) + " exceeds the file");
        }
        const std::string type(bytes.begin() + static_cast<std::ptrdiff_t>(at + 4),
                               bytes.begin() + static_cast<std::ptrdiff_t>(at + 8));
        const std::span<const std::uint8_t> data = bytes.subspan(at + 8, length);
        const std::uint32_t stored_crc = get_u32_be(bytes, at + 8 + length);
        if (crc32(bytes.subspan(at + 4, std::size_t{length} + 4)) != stored_crc) {
            return fail("CRC mismatch in chunk '" + type + "'");
        }
        at += std::size_t{length} + 12;

        if (!have_header && type != "IHDR") {
            return fail("first chunk is '" + type + "', expected 'IHDR'");
        }
        if (type == "IHDR") {
            if (have_header || length != 13) {
                return fail("malformed or repeated IHDR");
            }
            have_header = true;
            image.width = get_u32_be(data, 0);
            image.height = get_u32_be(data, 4);
            if (image.width == 0 || image.height == 0 || image.width > kMaxDimension ||
                image.height > kMaxDimension ||
                std::uint64_t{image.width} * image.height > kMaxPixels) {
                return fail("unsupported image size " + std::to_string(image.width) + "x" +
                            std::to_string(image.height));
            }
            if (data[8] != 8 || data[9] != 6 || data[10] != 0 || data[11] != 0 || data[12] != 0) {
                return fail("only 8-bit RGBA, non-interlaced images are supported");
            }
        } else if (type == "IDAT") {
            zlib.insert(zlib.end(), data.begin(), data.end());
        } else if (type == "IEND") {
            have_end = true;
            break;
        } else if (type == "PLTE") {
            continue; // suggested palette for a truecolour image; not needed
        } else if ((type[0] & 0x20) == 0) {
            return fail("unknown critical chunk '" + type + "'");
        }
    }
    if (!have_end) {
        return fail("missing IEND chunk");
    }

    const std::size_t row_bytes = std::size_t{image.width} * 4U;
    const std::size_t expected = (row_bytes + 1) * image.height;
    auto inflated = inflate_zlib(zlib, expected);
    if (!inflated) {
        return core::failure(inflated.error());
    }
    const std::vector<std::uint8_t>& raw = inflated.value();

    // Undo the row filters (bytes per pixel = 4).
    image.pixels.resize(image.byte_size());
    for (std::uint32_t y = 0; y < image.height; ++y) {
        const std::uint8_t filter = raw[y * (row_bytes + 1)];
        if (filter > 4) {
            return fail("invalid row filter " + std::to_string(filter) + " in row " +
                        std::to_string(y));
        }
        const std::uint8_t* in = raw.data() + y * (row_bytes + 1) + 1;
        std::uint8_t* out = image.pixels.data() + image.offset(0, y);
        const std::uint8_t* above = y > 0 ? out - row_bytes : nullptr;
        for (std::size_t i = 0; i < row_bytes; ++i) {
            const std::uint8_t a = i >= 4 ? out[i - 4] : 0;
            const std::uint8_t b = above != nullptr ? above[i] : 0;
            const std::uint8_t c = above != nullptr && i >= 4 ? above[i - 4] : 0;
            out[i] = static_cast<std::uint8_t>(in[i] + predictor(filter, a, b, c));
        }
    }
    return image;
}

core::Result<std::monostate, std::string> write_png(const std::filesystem::path& path,
                                                    const ImageRgba8& image) {
    auto encoded = encode_png(image);
    if (!encoded) {
        return core::failure(encoded.error());
    }
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!file) {
        return fail("cannot open '" + path.string() + "' for writing");
    }
    const std::vector<std::uint8_t>& bytes = encoded.value();
    const std::string buffer(bytes.begin(), bytes.end());
    file.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    if (!file) {
        return fail("write to '" + path.string() + "' failed");
    }
    return std::monostate{};
}

core::Result<ImageRgba8, std::string> read_png(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return fail("cannot open '" + path.string() + "'");
    }
    std::error_code size_error;
    const auto file_size = std::filesystem::file_size(path, size_error);
    if (size_error) {
        return fail("cannot read '" + path.string() + "'");
    }
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(file_size));
    file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    bytes.resize(static_cast<std::size_t>(file.gcount()));
    auto decoded = decode_png(bytes);
    if (!decoded) {
        return core::failure("'" + path.string() + "': " + decoded.error());
    }
    return decoded;
}

} // namespace orrery::render
