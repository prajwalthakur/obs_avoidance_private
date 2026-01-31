
#pragma once
#include <iostream>
#include <string>
#include "core/core/Utils.h"
#include "core/core/UuidGenerator.h"
class crUuid
{
    public:
        crUuid()=default;
        ~crUuid()=default;
        crUuid(const std::string& category,int value = -1);
        crUuid(const std::string& category,long int value = -1);
        static void setCategoryValue(const std::string& category, long int value = -1);

        // Getters
        const long int value() const;
        const std::string& category() const;
        //const std::string& toCategoryStr() const;
        void print() const ;
    private:
        void setCategoryValue();
    private:
        // Store the unique value
        long int mValue{-1};
        // Store the unique value in string format
        std::string mValueStr{"-1"};
        // Set the category
        std::string mCategory{"-1"};
        // store the unique value (value + category)
        std::string mValueCategoryStr{"-1"};
        // store the unique value in array
        //uint8_t mUuid[16];
        // Uuid Generator
        static inline UuidGenerator mGen;
};

namespace std
{
    inline std::size_t combine_hash(std::size_t seed, std::size_t h)
    {
        seed ^= h + 0x9e3779b9 + (seed << 6U) + (seed >> 2U);
        return seed;
    }

    template <>
    struct hash<crUuid>
    {
        std::size_t operator()(const crUuid& id) const noexcept
        {
            std::size_t h1 = std::hash<long int>{}(id.value());
            std::size_t h2 = std::hash<std::string>{}(id.category());
            return combine_hash(h1, h2);
        }
    };
}// namespace std