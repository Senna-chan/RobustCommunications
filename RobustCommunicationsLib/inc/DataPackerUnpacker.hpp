#ifndef _ROBUSTCOMMUNICATIONSDATAPACKERUNPACKER_H
#define _ROBUSTCOMMUNICATIONSDATAPACKERUNPACKER_H

#include <array>
#include <cstdint>
#include <cstring>
#include <string>


#include "Config.hpp"
namespace RobustCommunications{
class DataPackerUnpacker{
    public:
        std::array<uint8_t, MaxDataBytes> data = {0};
    private:
       size_t dataIndex = 0;
       template<typename Fun, typename...Ts>
       void sequential_foreach(Fun f, const Ts&... args) {
           (void) std::initializer_list<int>{
               ((void) f(args), 0)...
           };
       }
       
       void pack_value(std::integral auto x) {
           // little endian:
           for (auto i = 0u; i < sizeof(x); ++i) {
                data[dataIndex++] = (static_cast<uint8_t>((x >> (8u * i)) bitand 0xffu));
           }
       }
       
       template<std::integral ValueType>
       ValueType unpack_front() {
           auto ret = ValueType{};
           // TODO: do this properly:
           std::memcpy(&ret, &data[dataIndex], sizeof(ValueType));
           dataIndex += sizeof(ValueType);
           return ret;
       }
    public:
        template<typename...ValueTypes>
        void pack(ValueTypes... values) {
            sequential_foreach([&](auto value){pack_value(value);} ,values...);
        }
       
        template<typename...ValueTypes>
        std::tuple<ValueTypes...> 
        unpack() {
            return {unpack_front<ValueTypes>()...};
        }

        void setDataIndex(size_t index)
        {
            dataIndex = index;
        }

        void clear(){
            dataIndex = 0;
            data.fill(0);
        }
        
        uint8_t operator [] (size_t i) const {return data[i];}
};
};
#endif