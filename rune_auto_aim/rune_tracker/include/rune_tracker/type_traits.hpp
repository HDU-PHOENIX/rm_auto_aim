#pragma once

#include <type_traits>

// (using|bool|size_t)\s+([a-z]+_)+[a-z]\s

namespace rune {

    template<typename TType, TType Value>
    using IntegralConstant = std::integral_constant<TType, Value>;
    using TrueType = std::true_type;
    using FalseType = std::false_type;
    template<bool Value>
    using BoolConstant = std::bool_constant<Value>;

    template<typename... TBools>
    concept Conjunction = std::conjunction_v<TBools...>;
    template<typename... TBools>
    concept Disjunction = std::disjunction_v<TBools...>;
    template<typename TBool>
    concept Negation = std::negation_v<TBool>;
    template<typename TFrom, typename TTo>
    concept IsNothrowConvertible = std::is_nothrow_convertible_v<TFrom, TTo>;
    template<typename TType>
    using RemoveConst = std::remove_const_t<TType>;
    template<typename TType>
    using RemoveVolatile = std::remove_volatile_t<TType>;
    template<typename TType>
    using RemoveCV = std::remove_cv_t<TType>;
    template<typename TType>
    using AddConst = std::add_const_t<TType>;
    template<typename TType>
    using AddVolatile = std::add_volatile_t<TType>;
    template<typename TType>
    using AddCV = std::add_cv_t<TType>;
    template<typename TType>
    using RemoveReference = std::remove_reference_t<TType>;
    template<typename TType>
    using AddLvalueReference = std::add_lvalue_reference_t<TType>;
    template<typename TType>
    using AddRvalueReference = std::add_rvalue_reference_t<TType>;
    template<typename TType>
    using MakeSigned = std::make_signed_t<TType>;
    template<typename TType>
    using MakeUnsigned = std::make_unsigned_t<TType>;
    template<typename TType>
    using RemoveExtent = std::remove_extent_t<TType>;
    template<typename TType>
    using RemoveAllExtents = std::remove_all_extents_t<TType>;
    template<typename TType>
    using RemovePointer = std::remove_pointer_t<TType>;
    template<typename TType>
    using AddPointer = std::add_pointer_t<TType>;
    template<std::size_t VLength>
    using AlignedStorage = std::aligned_storage_t<VLength>;
    template<std::size_t VLength, typename... Types>
    using AlignedUnion = std::aligned_union_t<VLength, Types...>;
    template<typename TType>
    using Decay = std::decay_t<TType>;
    template<bool TCond, typename TType = void>
    using EnableIf = std::enable_if_t<TCond, TType>;
    template<bool TCond, typename TTrue, typename TFalse>
    using Conditional = std::conditional_t<TCond, TTrue, TFalse>;
    template<typename... TType>
    using CommonType = std::common_type_t<TType...>;
    template<typename TType>
    using UnderlyingType = std::underlying_type_t<TType>;
    template<typename TType>
    using ResultOf = std::result_of_t<TType>;
    template<typename TType>
    using Void = std::void_t<TType>;
    template<typename TType>
    concept IsSwappable = std::is_swappable_v<TType>;
    template<typename TType>
    concept IsNothrowSwappable = std::is_nothrow_swappable_v<TType>;
    template<typename TType, typename TUp>
    concept IsSwappableWith = std::is_swappable_with_v<TType, TUp>;
    template<typename TType, typename TUp>
    concept IsNothrowSwappableWith = std::is_nothrow_swappable_with_v<TType, TUp>;
    template<typename TType>
    using InvokeResult = std::invoke_result_t<TType>;
    template<typename TType>
    concept IsVoid = std::is_void_v<TType>;
    template<typename TType>
    concept IsNullPointer = std::is_null_pointer_v<TType>;
    template<typename TType>
    concept IsIntegral = std::is_integral_v<TType>;
    template<typename TType>
    concept IsFloatingPoint = std::is_floating_point_v<TType>;
    template<typename TType>
    concept IsArray = std::is_array_v<TType>;
    template<typename TType>
    concept IsPointer = std::is_pointer_v<TType>;
    template<typename TType>
    concept IsLvalueReference = std::is_lvalue_reference_v<TType>;
    template<typename TType>
    concept IsRvalueReference = std::is_rvalue_reference_v<TType>;
    template<typename TType>
    concept IsMemberObjectPointer = std::is_member_object_pointer_v<TType>;
    template<typename TType>
    concept IsMemberFunctionPointer = std::is_member_function_pointer_v<TType>;
    template<typename TType>
    concept IsEnum = std::is_enum_v<TType>;
    template<typename TType>
    concept IsUnion = std::is_union_v<TType>;
    template<typename TType>
    concept IsClass = std::is_class_v<TType>;
    template<typename TType>
    concept IsFunction = std::is_function_v<TType>;
    template<typename TType>
    concept IsReference = std::is_reference_v<TType>;
    template<typename TType>
    concept IsArithmetic = std::is_arithmetic_v<TType>;
    template<typename TType>
    concept IsFundamental = std::is_fundamental_v<TType>;
    template<typename TType>
    concept IsObject = std::is_object_v<TType>;
    template<typename TType>
    concept IsScalar = std::is_scalar_v<TType>;
    template<typename TType>
    concept IsCompound = std::is_compound_v<TType>;
    template<typename TType>
    concept IsMemberPointer = std::is_member_pointer_v<TType>;
    template<typename TType>
    concept IsConst = std::is_const_v<TType>;
    template<typename TType>
    concept IsVolatile = std::is_volatile_v<TType>;
    template<typename TType>
    concept IsTrivial = std::is_trivial_v<TType>;
    template<typename TType>
    concept IsTriviallyCopyable = std::is_trivially_copyable_v<TType>;
    template<typename TType>
    concept IsStandardLayout = std::is_standard_layout_v<TType>;
    template<typename TType>
    concept IsPod = std::is_pod_v<TType>;
    template<typename TType>
    concept IsLiteralType = std::is_literal_type_v<TType>;
    template<typename TType>
    concept IsEmpty = std::is_empty_v<TType>;
    template<typename TType>
    concept IsPolymorphic = std::is_polymorphic_v<TType>;
    template<typename TType>
    concept IsAbstract = std::is_abstract_v<TType>;
    template<typename TType>
    concept IsFinal = std::is_final_v<TType>;
    template<typename TType>
    concept IsSigned = std::is_signed_v<TType>;
    template<typename TType>
    concept IsUnsigned = std::is_unsigned_v<TType>;
    template<typename TType, typename... TArgs>
    concept IsConstructible = std::is_constructible_v<TType, TArgs...>;
    template<typename TType>
    concept IsDefaultConstructible = std::is_default_constructible_v<TType>;
    template<typename TType>
    concept IsCopyConstructible = std::is_copy_constructible_v<TType>;
    template<typename TType>
    concept IsMoveConstructible = std::is_move_constructible_v<TType>;
    template<typename TType, typename TUp>
    concept IsAssignable = std::is_assignable_v<TType, TUp>;
    template<typename TType>
    concept IsCopyAssignable = std::is_copy_assignable_v<TType>;
    template<typename TType>
    concept IsMoveAssignable = std::is_move_assignable_v<TType>;
    template<typename TType>
    concept IsDestructible = std::is_destructible_v<TType>;
    template<typename TType, typename... TArgs>
    concept IsTriviallyConstructible = std::is_trivially_constructible_v<TType, TArgs...>;
    template<typename TType>
    concept IsTriviallyDefaultConstructible = std::is_trivially_default_constructible_v<TType>;
    template<typename TType>
    concept IsTriviallyCopyConstructible = std::is_trivially_copy_constructible_v<TType>;
    template<typename TType>
    concept IsTriviallyMoveConstructible = std::is_trivially_move_constructible_v<TType>;
    template<typename TType, typename TUp>
    concept IsTriviallyAssignable = std::is_trivially_assignable_v<TType, TUp>;
    template<typename TType>
    concept IsTriviallyCopyAssignable = std::is_trivially_copy_assignable_v<TType>;
    template<typename TType>
    concept IsTriviallyMoveAssignable = std::is_trivially_move_assignable_v<TType>;
    template<typename TType>
    concept IsTriviallyDestructible = std::is_trivially_destructible_v<TType>;
    template<typename TType, typename... TArgs>
    concept IsNothrowConstructible = std::is_nothrow_constructible_v<TType, TArgs...>;
    template<typename TType>
    concept IsNothrowDefaultConstructible = std::is_nothrow_default_constructible_v<TType>;
    template<typename TType>
    concept IsNothrowCopyConstructible = std::is_nothrow_copy_constructible_v<TType>;
    template<typename TType>
    concept IsNothrowMoveConstructible = std::is_nothrow_move_constructible_v<TType>;
    template<typename TType, typename TUp>
    concept IsNothrowAssignable = std::is_nothrow_assignable_v<TType, TUp>;
    template<typename TType>
    concept IsNothrowCopyAssignable = std::is_nothrow_copy_assignable_v<TType>;
    template<typename TType>
    concept IsNothrowMoveAssignable = std::is_nothrow_move_assignable_v<TType>;
    template<typename TType>
    concept IsNothrowDestructible = std::is_nothrow_destructible_v<TType>;
    template<typename TType>
    concept HasVirtualDestructor = std::has_virtual_destructor_v<TType>;
    template<typename TType>
    inline constexpr std::size_t AlignmentOf = std::alignment_of_v<TType>;
    template<typename TType>
    inline constexpr std::size_t Rank = std::rank_v<TType>;
    template<typename TType>
    inline constexpr std::size_t Extent = std::extent_v<TType>;
    template<typename TType, typename TUp>
    concept IsSame = std::is_same_v<TType, TUp>;
    template<typename TBase, typename TDerived>
    concept IsBaseOf = std::is_base_of_v<TBase, TDerived>;
    template<typename TFrom, typename TTo>
    concept IsConvertible = std::is_convertible_v<TFrom, TTo>;
    template<typename TFunc, typename... TArgs>
    concept IsInvocable = std::is_invocable_v<TFunc, TArgs...>;
    template<typename TFunc, typename... TArgs>
    concept IsNothrowInvocable = std::is_nothrow_invocable_v<TFunc, TArgs...>;
    template<typename TRet, typename TFunc, typename... TArgs>
    concept IsInvocableR = std::is_invocable_r_v<TRet, TFunc, TArgs...>;
    template<typename TRet, typename TFunc, typename... TArgs>
    concept IsNothrowInvocableR = std::is_nothrow_invocable_r_v<TRet, TFunc, TArgs...>;
    template<typename TType>
    concept HasUniqueObjectRepresentation = std::has_unique_object_representations_v<TType>;
    template<typename TType>
    concept IsAggregate = std::is_aggregate_v<TType>;
    template<typename TType>
    using RemoveCVRef = std::remove_cvref_t<TType>;
    template<typename TType>
    using TypeIdentity = std::type_identity_t<TType>;
    template<typename TType>
    using UnwrapReference = std::unwrap_reference_t<TType>;
    template<typename TType>
    using UnwrapRefDecay = std::unwrap_ref_decay_t<TType>;
    template<typename TType>
    concept IsBoundedArray = std::is_bounded_array_v<TType>;
    template<typename TType>
    concept IsUnboundedArray = std::is_unbounded_array_v<TType>;
    template<typename... TType>
    using CommonReference = std::common_reference_t<TType...>;

    template<typename TType, template<typename...> typename TTemplate>
    struct is_instance_of : std::false_type {};
    template<typename... TArgs, template<typename...> typename TTemplate>
    struct is_instance_of<TTemplate<TArgs...>, TTemplate> : std::true_type {};
    template<typename TType, template<typename...> typename TTemplate>
    inline constexpr bool is_instance_of_v = is_instance_of<TType, TTemplate>::value;
    template<typename TType, template<typename...> typename TTemplate>
    concept IsInstanceOf = is_instance_of_v<TType, TTemplate>;

    template<template<typename...> typename TTemplate, typename TType>
    struct is_template_of : std::false_type {};
    template<template<typename...> typename TTemplate, typename... TArgs>
    struct is_template_of<TTemplate, TTemplate<TArgs...>> : std::true_type {};
    template<template<typename...> typename TTemplate, typename TType>
    inline constexpr bool is_template_of_v = is_template_of<TTemplate, TType>::value;
    template<template<typename...> typename TTemplate, typename TType>
    concept IsTemplateOf = is_template_of_v<TTemplate, TType>;
}  // namespace phoenix
