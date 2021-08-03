// Copyright 2020 Zoltán Rési
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTERNAL__SERIALIZATION_HPP_
#define INTERNAL__SERIALIZATION_HPP_

#include <vector>
#include <cstdint>
#include <algorithm>

namespace kuka_sunrise
{

int serializeNext(int integer_in, std::vector<std::uint8_t> & serialized_out)
{
  std::uint8_t * bytes = reinterpret_cast<std::uint8_t *>(&integer_in);
  auto it = serialized_out.end();
  serialized_out.insert(it, bytes, bytes + sizeof(int));
  std::reverse(it, serialized_out.end());
  // TODO(Zoltan Resi): assert that int is 4 bytes long
  // TODO(Zoltan Resi): check endiannes
  return sizeof(int);
}

int deserializeNext(const std::vector<std::uint8_t> & serialized_in, int & integer_out)
{
  if (serialized_in.size() < sizeof(int)) {
    // TODO(Zoltan Resi): error
  }
  std::vector<std::uint8_t> serialized_copy = serialized_in;
  integer_out = *(reinterpret_cast<int *>(serialized_copy.data()));
  return sizeof(int);
}

int serializeNext(double double_in, std::vector<std::uint8_t> & serialized_out)
{
  std::uint8_t * bytes = reinterpret_cast<std::uint8_t *>(&double_in);
  auto it = serialized_out.end();
  serialized_out.insert(serialized_out.end(), bytes, bytes + sizeof(double));
  std::reverse(it, serialized_out.end());
  // TODO(Zoltan Resi): assert that int is 4 bytes long
  // TODO(Zoltan Resi): check endiannes
  return sizeof(int);
}

int deserializeNext(const std::vector<std::uint8_t> & serialized_in, double & double_out)
{
  if (serialized_in.size() < sizeof(double)) {
    // TODO(Zoltan Resi): error
  }
  std::vector<std::uint8_t> serialized_copy = serialized_in;
  double_out = *(reinterpret_cast<int *>(serialized_copy.data()));
  return sizeof(int);
}

}  // namespace kuka_sunrise

#endif  // INTERNAL__SERIALIZATION_HPP_
