#include "UuidGenerator.h"

////////////////////////////////////////////////////////////////////////////////

void UuidGenerator::setCategoryValue(const std::string& category, long int value)
{
    const auto& ref = get();
    auto el = ref->mQueryMap.find(category);
    if(el == ref->mQueryMap.end())
    {
        ref->mQueryMap.insert({category,value});
    }
    else
    {
        el->second = value;
    }
}

////////////////////////////////////////////////////////////////////////////////

void UuidGenerator::resetValue(const std::string& category)
{
    const auto& ref = get();
    auto el = ref->mQueryMap.find(category);
    if(el != ref->mQueryMap.end())
    {
        ref->mQueryMap[category] = 0;
    }

}

////////////////////////////////////////////////////////////////////////////////

long int UuidGenerator::getUniqueValue(const std::string& category) const
{
    const auto& ref = get();
    auto el = ref->mQueryMap.find(category);
    if(el != ref->mQueryMap.end())
    {
        return el->second++;
    }
    else 
    {
        return 1;
    }
}

////////////////////////////////////////////////////////////////////////////////

long int UuidGenerator::checkCategoryAndReturnVal(const std::string& category) const
{
    const auto& ref = get();
    auto el = ref->mQueryMap.find(category);
    if(el == ref->mQueryMap.end())
    {
        return -1;
    }
    else
    {
        return el->second;
    }
}

