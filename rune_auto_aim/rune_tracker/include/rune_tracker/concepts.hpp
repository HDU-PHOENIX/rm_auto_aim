#pragma once

#include "type_traits.hpp"
#include <concepts>

// ^\s+concept\s+[^_]\w+

namespace rune {

template<typename TType, typename TOther>
concept SameAs = std::same_as<TType, TOther>;
template<typename TDerived, typename TBase>
concept DerivedFrom = std::derived_from<TDerived, TBase>;
template<typename TFrom, typename TTo>
concept ConvertibleTo = std::convertible_to<TFrom, TTo>;
template<typename TType, typename TOther>
concept CommonReferenceWith = std::common_reference_with<TType, TOther>;
template<typename TType, typename TOther>
concept CommonWith = std::common_with<TType, TOther>;
template<typename TType>
concept Integral = std::integral<TType>;
template<typename TType>
concept SignedIntegral = std::signed_integral<TType>;
template<typename TType>
concept UnsignedIntegral = std::unsigned_integral<TType>;
template<typename TType>
concept FloatingPoint = std::floating_point<TType>;
template<typename TLhs, typename TRhs>
concept AssignableFrom = std::assignable_from<TLhs, TRhs>;
template<typename TType>
concept Destructible = std::destructible<TType>;
template<typename TType, typename... TArgs>
concept ConstructibleFrom = std::constructible_from<TType, TArgs...>;
template<typename TType>
concept DefaultInitializable = std::default_initializable<TType>;
template<typename TType>
concept MoveConstructible = std::move_constructible<TType>;
template<typename TType>
concept copy_constructible = std::copy_constructible<TType>;
template<typename TType>
concept Swappable = std::swappable<TType>;
template<typename TType, typename TOther>
concept SwappableWith = std::swappable_with<TType, TOther>;
template<typename TType>
concept Movable = std::movable<TType>;
template<typename TType>
concept Copyable = std::copyable<TType>;
template<typename TType>
concept Semiregular = std::semiregular<TType>;
template<typename TType>
concept EqualityComparable = std::equality_comparable<TType>;
template<typename TType, typename TOther>
concept EqualityComparableWith = std::equality_comparable_with<TType, TOther>;
template<typename TType>
concept TotallyOrdered = std::totally_ordered<TType>;
template<typename TType, typename TOther>
concept TotallyOrderedWith = std::totally_ordered_with<TType, TOther>;
template<typename TType>
concept Regular = std::regular<TType>;
template<typename TFunc, typename... TArgs>
concept Invocable = std::invocable<TFunc, TArgs...>;
template<typename TFunc, typename... TArgs>
concept RegularInvocable = std::regular_invocable<TFunc, TArgs...>;
template<typename TFunc, typename... TArgs>
concept Predicate = std::predicate<TFunc, TArgs...>;
template<typename TRelation, typename TType, typename TOther>
concept Relation = std::relation<TRelation, TType, TOther>;
template<typename TRelation, typename TType, typename TOther>
concept EquivalenceRelation = std::equivalence_relation<TRelation, TType, TOther>;
template<typename TRelation, typename TType, typename TOther>
concept StrictWeakOrder = std::strict_weak_order<TRelation, TType, TOther>;

template<bool... TBools>
concept And = (true && ... && TBools);
template<bool... TBools>
concept Or = (false || ... || TBools);
template<bool TBool>
concept Not = (!TBool);

template<typename TType>
concept Deque = IsInstanceOf<TType, std::deque>;
template<typename TType>
concept ForwardList = IsInstanceOf<TType, std::forward_list>;
template<typename TType>
concept List = IsInstanceOf<TType, std::list>;
template<typename TType>
concept Vector = IsInstanceOf<TType, std::vector>;
template<typename TType>
concept Map = IsInstanceOf<TType, std::map>;
template<typename TType>
concept Multimap = IsInstanceOf<TType, std::multimap>;
template<typename TType>
concept Set = IsInstanceOf<TType, std::set>;
template<typename TType>
concept Multiset = IsInstanceOf<TType, std::multiset>;
template<typename TType>
concept UnorderedMap = IsInstanceOf<TType, std::unordered_map>;
template<typename TType>
concept UnorderedMultimap = IsInstanceOf<TType, std::unordered_multimap>;
template<typename TType>
concept UnorderedSet = IsInstanceOf<TType, std::unordered_set>;
template<typename TType>
concept UnorderedMultiset = IsInstanceOf<TType, std::unordered_multiset>;
template<typename TType>
concept Stack = IsInstanceOf<TType, std::stack>;
template<typename TType>
concept Queue = IsInstanceOf<TType, std::queue>;
template<typename TType>
concept PriorityQueue = IsInstanceOf<TType, std::priority_queue>;
template<typename TType>
concept StandardContainer = Or<Deque<TType>, ForwardList<TType>, List<TType>, Vector<TType>, Map<TType>, Multimap<TType>, Set<TType>, Multiset<TType>, UnorderedMap<TType>, UnorderedMultimap<TType>, UnorderedSet<TType>, UnorderedMultiset<TType>, Stack<TType>, Queue<TType>, PriorityQueue<TType>>;
} // namespace rune
