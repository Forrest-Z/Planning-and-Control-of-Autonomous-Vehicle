/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef CYBER_BASE_MACROS_H_
#define CYBER_BASE_MACROS_H_

#include <cstdlib>
#include <new>

#if __GNUC__ >= 3
#define likely(x) (__builtin_expect((x), 1))
#define unlikely(x) (__builtin_expect((x), 0))
#else
#define likely(x) (x)
#define unlikely(x) (x)
#endif

#define CACHELINE_SIZE 64

#define DEFINE_TYPE_TRAIT(name, func)                                                                                  \
  template <typename T>                                                                                                \
  class name                                                                                                           \
  {                                                                                                                    \
  private:                                                                                                             \
    template <typename Class>                                                                                          \
    static char Test(decltype(&Class::func)*);                                                                         \
    template <typename>                                                                                                \
    static int Test(...);                                                                                              \
                                                                                                                       \
  public:                                                                                                              \
    static constexpr bool value = sizeof(Test<T>(nullptr)) == 1;                                                       \
  };                                                                                                                   \
                                                                                                                       \
  template <typename T>                                                                                                \
  constexpr bool name<T>::value;

inline void cpu_relax()
{
  asm volatile("rep; nop" ::: "memory");
}

inline void* CheckedMalloc(size_t size)
{
  void* ptr = std::malloc(size);
  if (!ptr)
  {
    throw std::bad_alloc();
  }
  return ptr;
}

inline void* CheckedCalloc(size_t num, size_t size)
{
  void* ptr = std::calloc(num, size);
  if (!ptr)
  {
    throw std::bad_alloc();
  }
  return ptr;
}

#endif  // CYBER_BASE_MACROS_H_
