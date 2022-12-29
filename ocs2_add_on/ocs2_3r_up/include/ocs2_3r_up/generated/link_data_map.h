#ifndef IIT_BALLBOT_LINK_DATA_MAP_H_
#define IIT_BALLBOT_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace Ballbot {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[WORLD] = rhs[WORLD];
    data[LINK_1] = rhs[LINK_1];
    data[LINK_2] = rhs[LINK_2];
    data[GRIPPER] = rhs[GRIPPER];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[WORLD] = value;
    data[LINK_1] = value;
    data[LINK_2] = value;
    data[GRIPPER] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   world = "
    << map[WORLD]
    << "   link_1 = "
    << map[LINK_1]
    << "   link_2 = "
    << map[LINK_2]
    << "   gripper = "
    << map[GRIPPER]
    ;
    return out;
}

}
}
#endif
