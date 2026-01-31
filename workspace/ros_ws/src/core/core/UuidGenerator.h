#pragma once
#include <string>
#include <map>
#include <vector>
#include "core/core/Singleton.h"
#include "core/core/Random.h"
class UuidGenerator : public crSingleton<UuidGenerator>
{
    public:
        // set the category and value
        void setCategoryValue(const std::string& category, long int value);
        void resetValue(const std::string& category);
        long int getUniqueValue(const std::string& category) const;
        //std::string getUniqueKey(int length) const;
        long int checkCategoryAndReturnVal(const std::string& category) const;
    private:
        //long int mValue{0};
        std::string mKey;
        // Map
        std::map<std::string, long int> mQueryMap;
        // store the categories
        std::vector<std::string> mCategories;
        // Store the characters.
        // inline static const char mAlphaNum[] = "0123456789"
        //                                     "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        //                                     "abcdefghijklmnopqrstuvwxyz";    
        // Set the random number generator.
        //crRandomGenerator mGen;   
};