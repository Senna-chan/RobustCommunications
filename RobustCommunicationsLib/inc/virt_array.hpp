#ifndef VIRT_ARRAY_HPP
#define VIRT_ARRAY_HPP

// © Fiona Johanna Weber 2025
// license: AGPL version 3 or later

// Thanks cutie :)
#include <array>
#include <cstdint>
#include <stdexcept>
#include <type_traits>

namespace va {

namespace impl {
template<std::size_t N>
using smallest_index =
	std::conditional_t<(N < 1z <<  8), std::uint8_t,
	std::conditional_t<(N < 1z << 16), std::uint16_t,
	// std::conditional_t<(N < 1z << 32), std::uint32_t,
	std::size_t>>;
} // namespace impl

template<typename ValueType, std::size_t N, std::array<ValueType, N> Values>
class virt_ptr{
public:
	using index_type = impl::smallest_index<N>;
	
	constexpr virt_ptr(index_type index): index{index} {
		if (index >= N) {
			//throw std::out_of_range{"index out of bounds"};
		}
	}

	constexpr const auto* operator->() const {
		return &Values[index];
	}
	constexpr const auto& operator*() const {
		return Values[index];
	}

	static constexpr const std::array<ValueType, N>& get_full_data() {
		return Values;
	}
private:
	std::uint8_t index;
};

template<typename T, T... Values>
using make_virt_array = virt_ptr<T, sizeof...(Values), std::to_array({Values...})>;

} // namespace va


#endif

