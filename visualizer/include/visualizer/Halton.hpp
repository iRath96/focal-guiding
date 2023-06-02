#pragma once

#include <stdint.h>
#include <numeric>
#include <algorithm>

static inline float radicalInverse(uint64_t v, uint16_t dim);

struct HaltonPRNG {
    uint64_t index;
    uint16_t dimension;

    HaltonPRNG() : index(0), dimension(0) {}

    explicit HaltonPRNG(uint64_t index) : index(index), dimension(0) {}

    float operator()() {
        return radicalInverse(index, dimension++);
    }

    void advance() {
        index++;
        dimension = 0;
    }
};

template<int base>
inline float radicalInverse(uint64_t v) {
    const float invBase = 1 / float(base);
    uint64_t reversed = 0;
    float invBaseN = 1;

    while (v) {
        uint64_t next = v / base;
        uint64_t digit = v - next * base;
        reversed = reversed * base + digit;
        invBaseN *= invBase;
        v = next;
    }

    const float OneMinusEpsilon = std::nextafter(float(1), float(0)); /// @todo hack
    return std::min(reversed * invBaseN, OneMinusEpsilon);
}

static inline float radicalInverse(uint64_t v, uint16_t dim) {
    switch (dim) {
        case 0:
            return radicalInverse<2>(v);
        case 1:
            return radicalInverse<3>(v);
        case 2:
            return radicalInverse<5>(v);
        case 3:
            return radicalInverse<7>(v);
        case 4:
            return radicalInverse<11>(v);
        case 5:
            return radicalInverse<13>(v);
        case 6:
            return radicalInverse<17>(v);
        case 7:
            return radicalInverse<19>(v);
        case 8:
            return radicalInverse<23>(v);
        case 9:
            return radicalInverse<29>(v);
        case 10:
            return radicalInverse<31>(v);
        case 11:
            return radicalInverse<37>(v);
        case 12:
            return radicalInverse<41>(v);
        case 13:
            return radicalInverse<43>(v);
        case 14:
            return radicalInverse<47>(v);
        case 15:
            return radicalInverse<53>(v);
        case 16:
            return radicalInverse<59>(v);
        case 17:
            return radicalInverse<61>(v);
        case 18:
            return radicalInverse<67>(v);
        case 19:
            return radicalInverse<71>(v);
        case 20:
            return radicalInverse<73>(v);
        case 21:
            return radicalInverse<79>(v);
        case 22:
            return radicalInverse<83>(v);
        case 23:
            return radicalInverse<89>(v);
        case 24:
            return radicalInverse<97>(v);
        case 25:
            return radicalInverse<101>(v);
        case 26:
            return radicalInverse<103>(v);
        case 27:
            return radicalInverse<107>(v);
        case 28:
            return radicalInverse<109>(v);
        case 29:
            return radicalInverse<113>(v);
        case 30:
            return radicalInverse<127>(v);
        case 31:
            return radicalInverse<131>(v);
        case 32:
            return radicalInverse<137>(v);
        case 33:
            return radicalInverse<139>(v);
        case 34:
            return radicalInverse<149>(v);
        case 35:
            return radicalInverse<151>(v);
        case 36:
            return radicalInverse<157>(v);
        case 37:
            return radicalInverse<163>(v);
        case 38:
            return radicalInverse<167>(v);
        case 39:
            return radicalInverse<173>(v);
        case 40:
            return radicalInverse<179>(v);
        case 41:
            return radicalInverse<181>(v);
        case 42:
            return radicalInverse<191>(v);
        case 43:
            return radicalInverse<193>(v);
        case 44:
            return radicalInverse<197>(v);
        case 45:
            return radicalInverse<199>(v);
        case 46:
            return radicalInverse<211>(v);
        case 47:
            return radicalInverse<223>(v);
        case 48:
            return radicalInverse<227>(v);
        case 49:
            return radicalInverse<229>(v);
        case 50:
            return radicalInverse<233>(v);
        case 51:
            return radicalInverse<239>(v);
        case 52:
            return radicalInverse<241>(v);
        case 53:
            return radicalInverse<251>(v);
        case 54:
            return radicalInverse<257>(v);
        case 55:
            return radicalInverse<263>(v);
        case 56:
            return radicalInverse<269>(v);
        case 57:
            return radicalInverse<271>(v);
        case 58:
            return radicalInverse<277>(v);
        case 59:
            return radicalInverse<281>(v);
        case 60:
            return radicalInverse<283>(v);
        case 61:
            return radicalInverse<293>(v);
        case 62:
            return radicalInverse<307>(v);
        case 63:
            return radicalInverse<311>(v);
        case 64:
            return radicalInverse<313>(v);
        case 65:
            return radicalInverse<317>(v);
        case 66:
            return radicalInverse<331>(v);
        case 67:
            return radicalInverse<337>(v);
        case 68:
            return radicalInverse<347>(v);
        case 69:
            return radicalInverse<349>(v);
        case 70:
            return radicalInverse<353>(v);
        case 71:
            return radicalInverse<359>(v);
        case 72:
            return radicalInverse<367>(v);
        case 73:
            return radicalInverse<373>(v);
        case 74:
            return radicalInverse<379>(v);
        case 75:
            return radicalInverse<383>(v);
        case 76:
            return radicalInverse<389>(v);
        case 77:
            return radicalInverse<397>(v);
        case 78:
            return radicalInverse<401>(v);
        case 79:
            return radicalInverse<409>(v);
        case 80:
            return radicalInverse<419>(v);
        case 81:
            return radicalInverse<421>(v);
        case 82:
            return radicalInverse<431>(v);
        case 83:
            return radicalInverse<433>(v);
        case 84:
            return radicalInverse<439>(v);
        case 85:
            return radicalInverse<443>(v);
        case 86:
            return radicalInverse<449>(v);
        case 87:
            return radicalInverse<457>(v);
        case 88:
            return radicalInverse<461>(v);
        case 89:
            return radicalInverse<463>(v);
        case 90:
            return radicalInverse<467>(v);
        case 91:
            return radicalInverse<479>(v);
        case 92:
            return radicalInverse<487>(v);
        case 93:
            return radicalInverse<491>(v);
        case 94:
            return radicalInverse<499>(v);
        case 95:
            return radicalInverse<503>(v);
        case 96:
            return radicalInverse<509>(v);
        case 97:
            return radicalInverse<521>(v);
        case 98:
            return radicalInverse<523>(v);
        case 99:
            return radicalInverse<541>(v);
        case 100:
            return radicalInverse<547>(v);
        case 101:
            return radicalInverse<557>(v);
        case 102:
            return radicalInverse<563>(v);
        case 103:
            return radicalInverse<569>(v);
        case 104:
            return radicalInverse<571>(v);
        case 105:
            return radicalInverse<577>(v);
        case 106:
            return radicalInverse<587>(v);
        case 107:
            return radicalInverse<593>(v);
        case 108:
            return radicalInverse<599>(v);
        case 109:
            return radicalInverse<601>(v);
        case 110:
            return radicalInverse<607>(v);
        case 111:
            return radicalInverse<613>(v);
        case 112:
            return radicalInverse<617>(v);
        case 113:
            return radicalInverse<619>(v);
        case 114:
            return radicalInverse<631>(v);
        case 115:
            return radicalInverse<641>(v);
        case 116:
            return radicalInverse<643>(v);
        case 117:
            return radicalInverse<647>(v);
        case 118:
            return radicalInverse<653>(v);
        case 119:
            return radicalInverse<659>(v);
        case 120:
            return radicalInverse<661>(v);
        case 121:
            return radicalInverse<673>(v);
        case 122:
            return radicalInverse<677>(v);
        case 123:
            return radicalInverse<683>(v);
        case 124:
            return radicalInverse<691>(v);
        case 125:
            return radicalInverse<701>(v);
        case 126:
            return radicalInverse<709>(v);
        case 127:
            return radicalInverse<719>(v);
        default:
            exit(-1);
    }
}
