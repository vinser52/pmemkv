/*
 * Copyright 2017-2020, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "../pmemobj_engine.h"

#include <libpmemobj++/container/string.hpp>
#include <libpmemobj++/experimental/concurrent_map.hpp>
#include <libpmemobj++/experimental/v.hpp>
#include <libpmemobj++/persistent_ptr.hpp>
#include <libpmemobj++/shared_mutex.hpp>

#include <mutex>
#include <shared_mutex>

namespace pmem
{
namespace kv
{
namespace internal
{
namespace csmap
{

inline bool operator<(const pmem::obj::string &lhs, string_view rhs)
{
	return lhs.compare(0, lhs.size(), rhs.data(), rhs.size()) < 0;
}

inline bool operator<(string_view lhs, const pmem::obj::string &rhs)
{
	return rhs.compare(0, rhs.size(), lhs.data(), lhs.size()) > 0;
}

class hetero_less {
public:
	using is_transparent = void;

	template <typename M, typename U>
	bool operator()(const M &lhs, const U &rhs) const
	{
		return lhs < rhs;
	}
};

struct key_type : public pmem::obj::string {
	key_type() = default;
	key_type(const key_type &) = default;
	key_type(key_type &&) = default;
	key_type(string_view str) : pmem::obj::string(str.data(), str.size())
	{
	}
};

struct mapped_type {
	mapped_type() = default;

	mapped_type(const mapped_type &other) : val(other.val)
	{
	}

	mapped_type(mapped_type &&other) : val(std::move(other.val))
	{
	}

	mapped_type(const std::string &str) : val(str)
	{
	}

	mapped_type(string_view str) : val(str.data(), str.size())
	{
	}

	pmem::obj::shared_mutex mtx;
	pmem::obj::string val;
};

using map_t = pmem::obj::experimental::concurrent_map<key_type, mapped_type, hetero_less>;

} /* namespace csmap */
} /* namespace internal */

class csmap : public pmemobj_engine_base<internal::csmap::map_t> {
public:
	csmap(std::unique_ptr<internal::config> cfg);
	~csmap();

	csmap(const csmap &) = delete;
	csmap &operator=(const csmap &) = delete;

	std::string name() final;

	status count_all(std::size_t &cnt) final;

	status get_all(get_kv_callback *callback, void *arg) final;

	status exists(string_view key) final;

	status get(string_view key, get_v_callback *callback, void *arg) final;

	status put(string_view key, string_view value) final;

	status remove(string_view key) final;

private:
	using node_mutex_type = pmem::obj::shared_mutex;
	using mutex_type = std::shared_timed_mutex;
	using container_type = internal::csmap::map_t;

	void Recover();

	mutex_type mtx;
	container_type *container;
};

} /* namespace kv */
} /* namespace pmem */
