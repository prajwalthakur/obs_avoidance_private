#include "Uuid.h"

////////////////////////////////////////////////////////////////////////////////

// Uuid::Uuid(const std::string& category, int value)
//     :mValue(value)
// {
//     if(value == -1)
//         mValue = mGen.getUniqueValue(category);
//     mValueStr = std::to_string(mValue);
//     mCategory = category;
//     setCategoryValue(mCategory,mValue);
//     setCategoryValue();
// }

////////////////////////////////////////////////////////////////////////////////

Uuid::Uuid(const std::string& category, long int value)
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

void Uuid::setCategoryValue(const std::string& category, long int value)
{
    if(value<=-1)
        std::cerr<<"Uuid: value should be greater than -1";
    mGen.setCategoryValue(category,value);
    
}

////////////////////////////////////////////////////////////////////////////////

long int Uuid::value() const
{
    return mValue;
}

////////////////////////////////////////////////////////////////////////////////

const std::string& Uuid::category() const
{
    return mCategory;
}

////////////////////////////////////////////////////////////////////////////////

void Uuid::setCategoryValue()
{
    mValueCategoryStr = mValueStr + mCategory;
}

////////////////////////////////////////////////////////////////////////////////

void Uuid::print() const
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