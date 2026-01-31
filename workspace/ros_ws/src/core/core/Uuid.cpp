#include "core/core/Uuid.h"

////////////////////////////////////////////////////////////////////////////////

crUuid::crUuid(const std::string& category, int value)
    :mValue(value)
{
    if(value == -1)
        mValue = mGen.getUniqueValue(category);
    mValueStr = std::to_string(mValue);
    mCategory = category;
    setCategoryValue(mCategory,mValue);
    setCategoryValue();
}

////////////////////////////////////////////////////////////////////////////////

crUuid::crUuid(const std::string& category, long int value)
    :mValue(value)
{
    if(value == -1)
        mValue = mGen.getUniqueValue(category);
    mCategory = category;
    mValueStr = std::to_string(mValue);
    setCategoryValue(mCategory,mValue);
    setCategoryValue();
}

////////////////////////////////////////////////////////////////////////////////

void crUuid::setCategoryValue(const std::string& category, long int value)
{
    if(value<=-1)
        std::cerr<<"crUuid: value should be greater than -1";
    mGen.setCategoryValue(category,value);
    
}

////////////////////////////////////////////////////////////////////////////////

const long int crUuid::value() const
{
    return mValue;
}

////////////////////////////////////////////////////////////////////////////////

const std::string& crUuid::category() const
{
    return mCategory;
}

////////////////////////////////////////////////////////////////////////////////

void crUuid::setCategoryValue()
{
    mValueCategoryStr = mValueStr + mCategory;
}

////////////////////////////////////////////////////////////////////////////////

void crUuid::print() const
{
    if (mCategory.empty())
        fmt::print(
            "Category : (null) {:<12} Value : {:<5}\n",
            "",
            mValue
        );
    else
        fmt::print(
            "Category : {:<19} Value : {:<5}\n",
            mCategory,
            mValue
        );
}

////////////////////////////////////////////////////////////////////////////////