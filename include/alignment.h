/**
 * \file alignment.h
 * \author Zhihao Zhan (zhanzhihao_dt@163.com)
 * \brief memory alignment
 * \version 0.1
 * \date 2024-10-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef ALIGNMENT_H
#define ALIGNMENT_H

#include <initializer_list>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#ifndef EIGEN_ALIGNED_ALLOCATOR
#define EIGEN_ALIGNED_ALLOCATOR Eigen::aligned_allocator
#endif

// Equivalent to EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION but with support for
// initializer lists, which is a C++11 feature and not supported by the Eigen.
// The initializer list extension is inspired by Theia and StackOverflow code.
#define EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(...)                     \
  namespace std {                                                              \
  template <>                                                                  \
  class vector<__VA_ARGS__, std::allocator<__VA_ARGS__>>                       \
      : public vector<__VA_ARGS__, EIGEN_ALIGNED_ALLOCATOR<__VA_ARGS__>> {     \
    typedef vector<__VA_ARGS__, EIGEN_ALIGNED_ALLOCATOR<__VA_ARGS__>>          \
        vector_base;                                                           \
                                                                               \
  public:                                                                      \
    typedef __VA_ARGS__ value_type;                                            \
    typedef vector_base::allocator_type allocator_type;                        \
    typedef vector_base::size_type size_type;                                  \
    typedef vector_base::iterator iterator;                                    \
    explicit vector(const allocator_type &a = allocator_type())                \
        : vector_base(a) {}                                                    \
    template <typename InputIterator>                                          \
    vector(InputIterator first, InputIterator last,                            \
           const allocator_type &a = allocator_type())                         \
        : vector_base(first, last, a) {}                                       \
    vector(const vector &c) : vector_base(c) {}                                \
    explicit vector(size_type num, const value_type &val = value_type())       \
        : vector_base(num, val) {}                                             \
    vector(iterator start, iterator end) : vector_base(start, end) {}          \
    vector &operator=(const vector &x) {                                       \
      vector_base::operator=(x);                                               \
      return *this;                                                            \
    }                                                                          \
    vector(initializer_list<__VA_ARGS__> list)                                 \
        : vector_base(list.begin(), list.end()) {}                             \
  };                                                                           \
  } // namespace std

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Isometry3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Isometry3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Quaterniond)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Quaternionf)

#define EIGEN_STL_UMAP(KEY, VALUE)                                             \
  std::unordered_map<KEY, VALUE, std::hash<KEY>, std::equal_to<KEY>,           \
                     Eigen::aligned_allocator<std::pair<KEY const, VALUE>>>

#endif // ALIGNMENT_H