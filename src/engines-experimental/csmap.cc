/*
 * Copyright 2017-2019, Intel Corporation
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

#include "csmap.h"
#include "../out.h"

#include <unistd.h>

namespace pmem
{
namespace kv
{

csmap::csmap(std::unique_ptr<internal::config> cfg) : pmemobj_engine_base(cfg)
{
	LOG("Started ok");
	Recover();
}

csmap::~csmap()
{
	LOG("Stopped ok");
}

std::string csmap::name()
{
	return "csmap";
}

status csmap::count_all(std::size_t &cnt)
{
	LOG("count_all");
	check_outside_tx();
	cnt = container->size();

	return status::OK;
}

status csmap::get_all(get_kv_callback *callback, void *arg)
{
	LOG("get_all");
	check_outside_tx();
	for (auto it = container->begin(); it != container->end(); ++it) {
		auto ret = callback(it->first.c_str(), it->first.size(),
				    it->second.val.c_str(), it->second.val.size(), arg);

		if (ret != 0)
			return status::STOPPED_BY_CB;
	}

	return status::OK;
}

status csmap::exists(string_view key)
{
	LOG("exists for key=" << std::string(key.data(), key.size()));
	check_outside_tx();

	/*
	 * We take read lock for thread-safe methods (like contains) to synchronize with
	 * unsafe_erase() which sis not thread-safe.
	 */
	lock_type lock(mtx, false);
	return container->contains(key) ? status::OK : status::NOT_FOUND;
}

status csmap::get(string_view key, get_v_callback *callback, void *arg)
{
	LOG("get key=" << std::string(key.data(), key.size()));
	check_outside_tx();

	/*
	 * We take read lock for thread-safe methods (like find) to synchronize with
	 * unsafe_erase() which sis not thread-safe.
	 */
	lock_type lock(mtx, false);
	auto it = container->find(key);
	if (it != container->end()) {
		value_lock_type lock(it->second.mtx, false); // read lock
		callback(it->second.val.c_str(), it->second.val.size(), arg);
		return status::OK;
	}

	LOG("  key not found");
	return status::NOT_FOUND;
}

status csmap::put(string_view key, string_view value)
{
	LOG("put key=" << std::string(key.data(), key.size())
		       << ", value.size=" << std::to_string(value.size()));
	check_outside_tx();

	/*
	 * We take read lock for thread-safe methods (like emplace) to synchronize with
	 * unsafe_erase() which sis not thread-safe.
	 */
	lock_type lock(mtx, false);

    auto result = container->try_emplace(key, value);

	if (result.second == false) {
		auto &it = result.first;
		value_lock_type lock(it->second.mtx, true); // write lock
		pmem::obj::transaction::manual tx(pmpool);
		it->second.val.assign(value.data(), value.size());
		pmem::obj::transaction::commit();
	}

	return status::OK;
}

status csmap::remove(string_view key)
{
	LOG("remove key=" << std::string(key.data(), key.size()));
	check_outside_tx();
	lock_type lock(mtx, true);
	return container->unsafe_erase(key) > 0 ? status::OK : status::NOT_FOUND;
}

void csmap::Recover()
{
	if (!OID_IS_NULL(*root_oid)) {
		container = (pmem::kv::internal::csmap::map_t *)pmemobj_direct(*root_oid);
		container->runtime_initialize();
	} else {
		{ // tx scope
			pmem::obj::transaction::manual tx(pmpool);
			pmem::obj::transaction::snapshot(root_oid);
			*root_oid = pmem::obj::make_persistent<internal::csmap::map_t>()
					    .raw();
			pmem::obj::transaction::commit();
		}
		container = (pmem::kv::internal::csmap::map_t *)pmemobj_direct(*root_oid);
		container->runtime_initialize(true);
	}
}

} // namespace kv
} // namespace pmem
