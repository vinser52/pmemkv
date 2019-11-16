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

#ifndef LIBPMEMKV_HPP
#define LIBPMEMKV_HPP

#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

#if __cpp_lib_string_view
#include <string_view>
#endif

#include "libpmemkv.h"

/*! \file libpmemkv.hpp
	\brief Main C++ pmemkv public header.

	It contains all pmemkv public types, enums, classes with their functions and
	members.
*/

/*! \namespace pmem
	\brief Persistent memory namespace.

	It is a common namespace for all persistent memory C++ libraries
	For more information about pmem goto: http://pmem.io
*/
namespace pmem
{
/*! \namespace pmem::kv
	\brief Main pmemkv namespace.

	It contains all pmemkv public types, enums, classes with their functions and
   members. It is located within pmem namespace.
*/
namespace kv
{

#if __cpp_lib_string_view
using string_view = std::string_view;
#else
/*! \class string_view
	\brief Our brief std::string_view implementation.

	If C++17's std::string_view implementation is not available, this one is used
	to avoid unnecessary string copying.
*/
class string_view {
public:
	string_view() noexcept;
	string_view(const char *data, size_t size);

	template <typename Allocator>
	string_view(const std::basic_string<char, std::char_traits<char>, Allocator> &s);
	string_view(const char *data);

	string_view(const string_view &rhs) noexcept = default;
	string_view &operator=(const string_view &rhs) noexcept = default;

	const char *data() const noexcept;
	std::size_t size() const noexcept;

	int compare(const string_view &other) const noexcept;

private:
	const char *_data;
	std::size_t _size;
};
#endif

/**
 * The C++ idiomatic function type to use for callback using key-value pair.
 *
 * @param[in] key returned by callback item's key
 * @param[in] value returned by callback item's data
 */
typedef int get_kv_function(string_view key, string_view value);
/**
 * The C++ idiomatic function type to use for callback using only the value.
 * It is used only by non-range get() calls.
 *
 * @param[in] value returned by callback item's data
 */
typedef void get_v_function(string_view value);

/**
 * Key-value pair callback, C-style.
 */
using get_kv_callback = pmemkv_get_kv_callback;
/**
 * Value-only callback, C-style.
 */
using get_v_callback = pmemkv_get_v_callback;

/*! \enum status
	\brief Status returned by pmemkv functions.

	Each function, except for db::close() and pmem::kv::errormsg(), returns one of the
	following status codes.
*/
enum class status {
	OK = PMEMKV_STATUS_OK,			     /**< no error */
	UNKNOWN_ERROR = PMEMKV_STATUS_UNKNOWN_ERROR, /**< unknown error */
	NOT_FOUND = PMEMKV_STATUS_NOT_FOUND, /**< record (or config item) not found */
	NOT_SUPPORTED = PMEMKV_STATUS_NOT_SUPPORTED, /**< function is not implemented by
							current engine */
	INVALID_ARGUMENT = PMEMKV_STATUS_INVALID_ARGUMENT, /**< argument to function has
							wrong value */
	CONFIG_PARSING_ERROR =
		PMEMKV_STATUS_CONFIG_PARSING_ERROR, /**< parsing data to config failed */
	CONFIG_TYPE_ERROR =
		PMEMKV_STATUS_CONFIG_TYPE_ERROR, /**< config item has different type than
							expected */
	STOPPED_BY_CB = PMEMKV_STATUS_STOPPED_BY_CB, /**< iteration was stopped by user's
							callback */
	OUT_OF_MEMORY =
		PMEMKV_STATUS_OUT_OF_MEMORY, /**< operation failed because there is not
						enough memory (or space on the device) */
	WRONG_ENGINE_NAME =
		PMEMKV_STATUS_WRONG_ENGINE_NAME, /**< engine name does not match any
							available engine */
	TRANSACTION_SCOPE_ERROR =
		PMEMKV_STATUS_TRANSACTION_SCOPE_ERROR, /**< an error with the scope of the
							libpmemobj transaction */
};

/*! \class config
	\brief Holds configuration parameters for engines.

	It stores mappings of keys (strings) to values. A value can be:
		* uint64_t,
		* int64_t,
		* string,
		* binary data,
		* pointer to an object (with accompanying deleter function).

	It also delivers methods to store and read configuration items provided by
	a user. Once the configuration object is set (with all required
	parameters),pmemkv_open it can be passed to db::open() method.

	List of options which are required by pmemkv database is specific to an engine.
	Every engine has documented all supported config parameters (please see
	[libpmemkv(7)](https://pmem.io/pmemkv/master/manpages/libpmemkv.7.html) for
	details).
*/
class config {
public:
	config() noexcept;
	explicit config(pmemkv_config *cfg) noexcept;

	~config();

	config(const config &other) = delete;
	config(config &&other) noexcept;

	config &operator=(const config &other) = delete;
	config &operator=(config &&other) noexcept;

	template <typename T>
	status put_data(const std::string &key, const T *value,
			const std::size_t number = 1) noexcept;
	template <typename T>
	status put_object(
			const std::string &key, T *value,
			void (*deleter)(void *) = [](T *value) { delete value; }) noexcept;
	status put_uint64(const std::string &key, std::uint64_t value) noexcept;
	status put_int64(const std::string &key, std::int64_t value) noexcept;
	status put_string(const std::string &key, const std::string &value) noexcept;

	template <typename T>
	status get_data(const std::string &key, T *&value, std::size_t &number) const
		noexcept;
	template <typename T>
	status get_object(const std::string &key, T *&value) const noexcept;

	status get_uint64(const std::string &key, std::uint64_t &value) const noexcept;
	status get_int64(const std::string &key, std::int64_t &value) const noexcept;
	status get_string(const std::string &key, std::string &value) const noexcept;

	pmemkv_config *release() noexcept;

private:
	int init() noexcept;

	pmemkv_config *_config;
};

/*! \class db
	\brief Main pmemkv class, it provides functions to operate on data in database.

	Database class for creating, opening and closing pmemkv's data file.
	It provides functions to write, read & remove data, count elements stored
	and check for existence of an element based on its key.
*/
class db {
public:
	db() noexcept;
	~db();

	db(const db &other) = delete;
	db(db &&other) noexcept;

	db &operator=(const db &other) = delete;
	db &operator=(db &&other) noexcept;

	status open(const std::string &engine_name) noexcept;
	status open(const std::string &engine_name, config &&cfg) noexcept;

	void close() noexcept;

	status count_all(std::size_t &cnt) noexcept;
	status count_above(string_view key, std::size_t &cnt) noexcept;
	status count_below(string_view key, std::size_t &cnt) noexcept;
	status count_between(string_view key1, string_view key2,
			     std::size_t &cnt) noexcept;

	status get_all(get_kv_callback *callback, void *arg) noexcept;
	status get_all(std::function<get_kv_function> f) noexcept;

	status get_above(string_view key, get_kv_callback *callback, void *arg) noexcept;
	status get_above(string_view key, std::function<get_kv_function> f) noexcept;

	status get_below(string_view key, get_kv_callback *callback, void *arg) noexcept;
	status get_below(string_view key, std::function<get_kv_function> f) noexcept;

	status get_between(string_view key1, string_view key2, get_kv_callback *callback,
			   void *arg) noexcept;
	status get_between(string_view key1, string_view key2,
			   std::function<get_kv_function> f) noexcept;

	status exists(string_view key) noexcept;

	status get(string_view key, get_v_callback *callback, void *arg) noexcept;
	status get(string_view key, std::function<get_v_function> f) noexcept;
	status get(string_view key, std::string *value) noexcept;

	status put(string_view key, string_view value) noexcept;
	status remove(string_view key) noexcept;

private:
	pmemkv_db *_db;
};

/**
 * Default constructor with uninitialized config.
 */
inline config::config() noexcept
{
	this->_config = nullptr;
}

/**
 * Move constructor. Initializes config with another config.
 * Ownership is transferred to config class.
 */
inline config::config(config &&other) noexcept
{
	this->_config = other._config;
	other._config = nullptr;
}

/**
 * Move assignment operator. Deletes previous config and replaces
 * it with another config. Ownership is transferred to config class.
 */
inline config &config::operator=(config &&other) noexcept
{
	if (this->_config)
		pmemkv_config_delete(this->_config);

	this->_config = other._config;
	other._config = nullptr;

	return *this;
}

/**
 * Creates config from pointer to pmemkv_config.
 * Ownership is transferred to config class.
 */
inline config::config(pmemkv_config *cfg) noexcept
{
	this->_config = cfg;
}

/**
 * Default destructor. Deletes config if initialized.
 */
inline config::~config()
{
	if (this->_config)
		pmemkv_config_delete(this->_config);
}

/**
 * Initialization function for config.
 * It's lazy initialized and called within all put functions.
 *
 * @return int initialization result; 0 on success
 */
inline int config::init() noexcept
{
	if (this->_config == nullptr) {
		this->_config = pmemkv_config_new();

		if (this->_config == nullptr)
			return 1;
	}

	return 0;
}

/**
 * Puts binary data pointed by value, of type T, with count of elements
 * to a config. Count parameter is useful for putting arrays of data.
 *
 * @param[in] key The string representing config item's name.
 * @param[in] value The pointer to data.
 * @param[in] count The count of elements stored under reference.
 *
 * @return pmem::kv::status
 */
template <typename T>
inline status config::put_data(const std::string &key, const T *value,
			       const std::size_t count) noexcept
{
	if (init() != 0)
		return status::UNKNOWN_ERROR;

	return static_cast<status>(pmemkv_config_put_data(
		this->_config, key.data(), (void *)value, count * sizeof(T)));
}

/**
 * Puts object pointed by value, of type T, with given destructor
 * to a config.
 *
 * @param[in] key The string representing config item's name.
 * @param[in] value The pointer to object.
 * @param[in] deleter The object's destructor function.
 *
 * @return pmem::kv::status
 */
template <typename T>
inline status config::put_object(const std::string &key, T *value,
				 void (*deleter)(void *)) noexcept
{
	if (init() != 0)
		return status::UNKNOWN_ERROR;

	return static_cast<status>(pmemkv_config_put_object(this->_config, key.data(),
							    (void *)value, deleter));
}

/**
 * Puts std::uint64_t value to a config.
 *
 * @param[in] key The string representing config item's name.
 * @param[in] value The std::uint64_t value.
 *
 * @return pmem::kv::status
 */
inline status config::put_uint64(const std::string &key, std::uint64_t value) noexcept
{
	if (init() != 0)
		return status::UNKNOWN_ERROR;

	return static_cast<status>(
		pmemkv_config_put_uint64(this->_config, key.data(), value));
}

/**
 * Puts std::int64_t value to a config.
 *
 * @param[in] key The string representing config item's name.
 * @param[in] value The std::int64_t value.
 *
 * @return pmem::kv::status
 */
inline status config::put_int64(const std::string &key, std::int64_t value) noexcept
{
	if (init() != 0)
		return status::UNKNOWN_ERROR;

	return static_cast<status>(
		pmemkv_config_put_int64(this->_config, key.data(), value));
}

/**
 * Puts string value to a config.
 *
 * @param[in] key The string representing config item's name.
 * @param[in] value The string value.
 *
 * @return pmem::kv::status
 */
inline status config::put_string(const std::string &key,
				 const std::string &value) noexcept
{
	if (init() != 0)
		return status::UNKNOWN_ERROR;

	return static_cast<status>(
		pmemkv_config_put_string(this->_config, key.data(), value.data()));
}

/**
 * Gets object from a config item with key name and copies it
 * into T object value.
 *
 * @param[in] key The string representing config item's name.
 * @param[out] value The pointer to data.
 * @param[out] count The count of elements stored under reference.
 *
 * @return pmem::kv::status
 */
template <typename T>
inline status config::get_data(const std::string &key, T *&value,
			       std::size_t &count) const noexcept
{
	if (this->_config == nullptr)
		return status::NOT_FOUND;

	std::size_t size;
	auto s = static_cast<status>(pmemkv_config_get_data(
		this->_config, key.data(), (const void **)&value, &size));

	if (s != status::OK)
		return s;

	count = size / sizeof(T);

	return status::OK;
}

/**
 * Gets binary data from a config item with key name and
 * assigns pointer to T object value.
 *
 * @param[in] key The string representing config item's name.
 * @param[out] value The pointer to object.
 *
 * @return pmem::kv::status
 */
template <typename T>
inline status config::get_object(const std::string &key, T *&value) const noexcept
{
	if (this->_config == nullptr)
		return status::NOT_FOUND;

	auto s = static_cast<status>(
		pmemkv_config_get_object(this->_config, key.data(), (void **)&value));

	return s;
}

/**
 * Gets std::uint64_t value from a config item with key name.
 *
 * @param[in] key The string representing config item's name.
 * @param[out] value The std::uint64_t value.
 *
 * @return pmem::kv::status
 */
inline status config::get_uint64(const std::string &key, std::uint64_t &value) const
	noexcept
{
	if (this->_config == nullptr)
		return status::NOT_FOUND;

	return static_cast<status>(
		pmemkv_config_get_uint64(this->_config, key.data(), &value));
}

/**
 * Gets std::int64_t value from a config item with key name.
 *
 * @param[in] key The string representing config item's name.
 * @param[out] value The std::int64_t value.
 *
 * @return pmem::kv::status
 */
inline status config::get_int64(const std::string &key, std::int64_t &value) const
	noexcept
{
	if (this->_config == nullptr)
		return status::NOT_FOUND;

	return static_cast<status>(
		pmemkv_config_get_int64(this->_config, key.data(), &value));
}

/**
 * Gets string value from a config item with key name.
 *
 * @param[in] key The string representing config item's name.
 * @param[out] value The string value.
 *
 * @return pmem::kv::status
 */
inline status config::get_string(const std::string &key, std::string &value) const
	noexcept
{
	if (this->_config == nullptr)
		return status::NOT_FOUND;

	const char *data;

	auto s = static_cast<status>(
		pmemkv_config_get_string(this->_config, key.data(), &data));

	if (s != status::OK)
		return s;

	value = data;

	return status::OK;
}

/**
 * Similarly to std::unique_ptr::release it passes the ownership
 * of underlying pmemkv_config variable and sets it to nullptr.
 *
 * @return handle to pmemkv_config
 */
inline pmemkv_config *config::release() noexcept
{
	auto c = this->_config;
	this->_config = nullptr;
	return c;
}

#if !__cpp_lib_string_view
/**
 * Default constructor with empty data.
 */
inline string_view::string_view() noexcept : _data(""), _size(0)
{
}

/**
 * Constructor initialized by *data* and its *size*.
 *
 * @param[in] data pointer to the C-like string (char *) to initialize with,
 *				it can contain null characters
 * @param[in] size length of the given data
 */
inline string_view::string_view(const char *data, size_t size) : _data(data), _size(size)
{
}

/**
 * Constructor initialized by the basic_string *s*.
 *
 * @param[in] s reference to the basic_string to initialize with
 */      
template <typename Allocator>
inline string_view::string_view(
	const std::basic_string<char, std::char_traits<char>, Allocator> &s)
    : _data(s.c_str()), _size(s.size())
{
}

/**
 * Constructor initialized by *data*. Size of the data will be set
 * using std::char_traits<char>::length().
 *
 * @param[in] data pointer to C-like string (char *) to initialize with,
 *				it has to end with the terminating null character
 */
inline string_view::string_view(const char *data)
    : _data(data), _size(std::char_traits<char>::length(data))
{
}

/**
 * Returns pointer to data stored in this pmem::kv::string_view. It may not contain
 * the terminating null character.
 *
 * @return pointer to C-like string (char *), it may not end with null character
 */
inline const char *string_view::data() const noexcept
{
	return _data;
}

/**
 * Returns count of characters stored in this pmem::kv::string_view data.
 *
 * @return pointer to C-like string (char *), it may not end with null character
 */
inline std::size_t string_view::size() const noexcept
{
	return _size;
}

/**
 * Compares this string_view with other. Works in the same way as
 * std::basic_string::compare.
 *
 * @return 0 if both character sequences compare equal,
 *			positive value if this is lexicographically greater than other,
 *			negative value if this is lexicographically less than other.
 */
inline int string_view::compare(const string_view &other) const noexcept
{
	int ret = std::char_traits<char>::compare(data(), other.data(),
						  std::min(size(), other.size()));
	if (ret != 0)
		return ret;
	if (size() < other.size())
		return -1;
	if (size() > other.size())
		return 1;
	return 0;
}

inline bool operator<(const string_view &lhs, const string_view &rhs) noexcept
{
	return lhs.compare(rhs) == -1;
}
#endif

/*
 * All functions which will be called by C code must be declared as extern "C"
 * to ensure they have C linkage. It is needed because it is possible that
 * C and C++ functions use different calling conventions.
 */
extern "C" {
static inline int call_get_kv_function(const char *key, size_t keybytes,
				       const char *value, size_t valuebytes, void *arg)
{
	return (*reinterpret_cast<std::function<get_kv_function> *>(arg))(
		string_view(key, keybytes), string_view(value, valuebytes));
}

static inline void call_get_v_function(const char *value, size_t valuebytes, void *arg)
{
	(*reinterpret_cast<std::function<get_v_function> *>(arg))(
		string_view(value, valuebytes));
}

static inline void call_get_copy(const char *v, size_t vb, void *arg)
{
	auto c = reinterpret_cast<std::string *>(arg);
	c->assign(v, vb);
}
}

/**
 * Default constructor with uninitialized database.
 */
inline db::db() noexcept
{
	this->_db = nullptr;
}

/**
 * Move constructor. Initializes database with another database.
 * Ownership is being transferred to a class that move constructor was called on.
 *
 * @param[in] other another database, to be moved from
 */
inline db::db(db &&other) noexcept
{
	this->_db = other._db;
	other._db = nullptr;
}

/**
 * Move assignment operator. Deletes previous database and replaces
 * it with another database. Ownership is being transferred to a class that
 * assign operator was called on.
 *
 * @param[in] other another database, to be assigned from
 */
inline db &db::operator=(db &&other) noexcept
{
	close();

	std::swap(this->_db, other._db);

	return *this;
}

/**
 * Opens the pmemkv database without any configuration parameters.
 *
 * @param[in] engine_name name of the engine to work with
 *
 * @return pmem::kv::status
 */
inline status db::open(const std::string &engine_name) noexcept
{
	return static_cast<status>(
		pmemkv_open(engine_name.c_str(), nullptr, &(this->_db)));
}

/**
 * Opens the pmemkv database with specified config.
 *
 * @param[in] engine_name name of the engine to work with
 * @param[in] cfg pmem::kv::config with parameters specified for the engine
 *
 * @return pmem::kv::status
 */
inline status db::open(const std::string &engine_name, config &&cfg) noexcept
{
	return static_cast<status>(
		pmemkv_open(engine_name.c_str(), cfg.release(), &(this->_db)));
}

/**
 * Closes pmemkv database.
 */
inline void db::close() noexcept
{
	if (this->_db != nullptr)
		pmemkv_close(this->_db);

	this->_db = nullptr;
}

/**
 * Default destructor. Closes pmemkv database.
 */
inline db::~db()
{
	close();
}

/**
 * It returns number of currently stored elements in pmem::kv::db.
 *
 * @param[out] cnt number of records stored in pmem::kv::db.
 *
 * @return pmem::kv::status
 */
inline status db::count_all(std::size_t &cnt) noexcept
{
	return static_cast<status>(pmemkv_count_all(this->_db, &cnt));
}

/**
 * It returns number of currently stored elements in pmem::kv::db, whose keys
 * are greater than the given *key*.
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key sets the lower bound of counting
 * @param[out] cnt number of records in pmem::kv::db matching query
 *
 * @return pmem::kv::status
 */
inline status db::count_above(string_view key, std::size_t &cnt) noexcept
{
	return static_cast<status>(
		pmemkv_count_above(this->_db, key.data(), key.size(), &cnt));
}

/**
 * It returns number of currently stored elements in pmem::kv::db, whose keys
 * are less than the given *key*.
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key sets the upper bound of counting
 * @param[out] cnt number of records in pmem::kv::db matching query
 *
 * @return pmem::kv::status
 */
inline status db::count_below(string_view key, std::size_t &cnt) noexcept
{
	return static_cast<status>(
		pmemkv_count_below(this->_db, key.data(), key.size(), &cnt));
}

/**
 * It returns number of currently stored elements in pmem::kv::db, whose keys
 * are greater than the *key1* and less than the *key2*.
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key1 sets the lower bound of counting
 * @param[in] key2 sets the upper bound of counting
 * @param[out] cnt number of records in pmem::kv::db matching query
 *
 * @return pmem::kv::status
 */
inline status db::count_between(string_view key1, string_view key2,
				std::size_t &cnt) noexcept
{
	return static_cast<status>(pmemkv_count_between(
		this->_db, key1.data(), key1.size(), key2.data(), key2.size(), &cnt));
}

/**
 * Executes (C-like) *callback* function for every record stored in pmem::kv::db.
 * Arguments passed to the callback function are: pointer to a key, size of the
 * key, pointer to a value, size of the value and *arg* specified by the user.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * @param[in] callback function to be called for every element stored in db
 * @param[in] arg additional arguments to be passed to callback
 *
 * @return pmem::kv::status
 */
inline status db::get_all(get_kv_callback *callback, void *arg) noexcept
{
	return static_cast<status>(pmemkv_get_all(this->_db, callback, arg));
}

/**
 * Executes function for every record stored in pmem::kv::db.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * @param[in] f function called for each returned element, it is called with params:
 *				key and value
 *
 * @return pmem::kv::status
 */
inline status db::get_all(std::function<get_kv_function> f) noexcept
{
	return static_cast<status>(pmemkv_get_all(this->_db, call_get_kv_function, &f));
}

/**
 * Executes (C-like) callback function for every record stored in pmem::kv::db,
 * whose keys are greater than the given *key*.
 * Arguments passed to the callback function are: pointer to a key, size of the
 * key, pointer to a value, size of the value and *arg* specified by the user.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key sets the lower bound for querying
 * @param[in] callback function to be called for each returned element
 * @param[in] arg additional arguments to be passed to callback
 *
 * @return pmem::kv::status
 */
inline status db::get_above(string_view key, get_kv_callback *callback,
			    void *arg) noexcept
{
	return static_cast<status>(
		pmemkv_get_above(this->_db, key.data(), key.size(), callback, arg));
}

/**
 * Executes function for every record stored in pmem::kv::db, whose keys are
 * greater than the given *key*.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key sets the lower bound for querying
 * @param[in] f function called for each returned element, it is called with params:
 *				key and value
 *
 * @return pmem::kv::status
 */
inline status db::get_above(string_view key, std::function<get_kv_function> f) noexcept
{
	return static_cast<status>(pmemkv_get_above(this->_db, key.data(), key.size(),
						    call_get_kv_function, &f));
}

/**
 * Executes (C-like) callback function for every record stored in pmem::kv::db,
 * whose keys are less than the given *key*.
 * Arguments passed to the callback function are: pointer to a key, size of the
 * key, pointer to a value, size of the value and *arg* specified by the user.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key sets the upper bound for querying
 * @param[in] callback function to be called for each returned element
 * @param[in] arg additional arguments to be passed to callback
 *
 * @return pmem::kv::status
 */
inline status db::get_below(string_view key, get_kv_callback *callback,
			    void *arg) noexcept
{
	return static_cast<status>(
		pmemkv_get_below(this->_db, key.data(), key.size(), callback, arg));
}

/**
 * Executes function for every record stored in pmem::kv::db, whose keys are
 * less than the given *key*.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key sets the upper bound for querying
 * @param[in] f function called for each returned element, it is called with params:
 *				key and value
 *
 * @return pmem::kv::status
 */
inline status db::get_below(string_view key, std::function<get_kv_function> f) noexcept
{
	return static_cast<status>(pmemkv_get_below(this->_db, key.data(), key.size(),
						    call_get_kv_function, &f));
}

/**
 * Executes (C-like) callback function for every record stored in pmem::kv::db,
 * whose keys are greater than the *key1* and less than the *key2*.
 * Arguments passed to the callback function are: pointer to a key, size of the
 * key, pointer to a value, size of the value and *arg* specified by the user.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key1 sets the lower bound for querying
 * @param[in] key2 sets the upper bound for querying
 * @param[in] callback function to be called for each returned element
 * @param[in] arg additional arguments to be passed to callback
 *
 * @return pmem::kv::status
 */
inline status db::get_between(string_view key1, string_view key2,
			      get_kv_callback *callback, void *arg) noexcept
{
	return static_cast<status>(pmemkv_get_between(this->_db, key1.data(), key1.size(),
						      key2.data(), key2.size(), callback,
						      arg));
}
/**
 * Executes function for every record stored in pmem::kv::db, whose keys
 * are greater than the *key1* and less than the *key2*.
 * Callback can stop iteration by returning non-zero value. In that case *get_all()*
 * returns pmem::kv::status::STOPPED_BY_CB. Returning 0 continues iteration.
 *
 * Keys are sorted in lexicographical order (see std::lexicographical_compare).
 *
 * @param[in] key1 sets the lower bound for querying
 * @param[in] key2 sets the upper bound for querying
 * @param[in] f function called for each returned element, it is called with params:
 *				key and value
 *
 * @return pmem::kv::status
 */
inline status db::get_between(string_view key1, string_view key2,
			      std::function<get_kv_function> f) noexcept
{
	return static_cast<status>(pmemkv_get_between(this->_db, key1.data(), key1.size(),
						      key2.data(), key2.size(),
						      call_get_kv_function, &f));
}

/**
 * Checks existence of record with given *key*. If record is present
 * pmem::kv::status::OK is returned, otherwise pmem::kv::status::NOT_FOUND
 * is returned. Other possible return values are described in pmem::kv::status.
 *
 * @param[in] key record's key to query for in database
 *
 * @return pmem::kv::status
 */
inline status db::exists(string_view key) noexcept
{
	return static_cast<status>(pmemkv_exists(this->_db, key.data(), key.size()));
}

/**
 * Executes (C-like) *callback* function for record with given *key*. If
 * record is present and no error occurred, the function returns
 * pmem::kv::status::OK. If record does not exist pmem::kv::status::NOT_FOUND
 * is returned. Other possible return values are described in pmem::kv::status.
 * *Callback* is called with the following parameters:
 * pointer to a value, size of the value and *arg* specified by the user.
 * This function is guaranteed to be implemented by all engines.
 *
 * @param[in] key record's key to query for
 * @param[in] callback function to be called for returned element
 * @param[in] arg additional arguments to be passed to callback
 *
 * @return pmem::kv::status
 */
inline status db::get(string_view key, get_v_callback *callback, void *arg) noexcept
{
	return static_cast<status>(
		pmemkv_get(this->_db, key.data(), key.size(), callback, arg));
}

/**
 * Executes function for record with given *key*. If record is present and
 * no error occurred the function returns pmem::kv::status::OK. If record does
 * not exist pmem::kv::status::NOT_FOUND is returned.
 *
 * @param[in] key record's key to query for
 * @param[in] f function called for returned element, it is called with only
 *				one param - value (key is known)
 *
 * @return pmem::kv::status
 */
inline status db::get(string_view key, std::function<get_v_function> f) noexcept
{
	return static_cast<status>(
		pmemkv_get(this->_db, key.data(), key.size(), call_get_v_function, &f));
}

/**
 * Gets value copy of record with given *key*. In absence of any errors,
 * pmem::kv::status::OK is returned.
 * This function is guaranteed to be implemented by all engines.
 *
 * @param[in] key record's key to query for
 * @param[out] value stores returned copy of the data
 *
 * @return pmem::kv::status
 */
inline status db::get(string_view key, std::string *value) noexcept
{
	return static_cast<status>(
		pmemkv_get(this->_db, key.data(), key.size(), call_get_copy, value));
}

/**
 * Inserts a key-value pair into pmemkv database.
 * This function is guaranteed to be implemented by all engines.
 *
 * @param[in] key record's key; record will be put into database under its name
 * @param[in] value data to be inserted into this new database record
 *
 * @return pmem::kv::status
 */
inline status db::put(string_view key, string_view value) noexcept
{
	return static_cast<status>(pmemkv_put(this->_db, key.data(), key.size(),
					      value.data(), value.size()));
}

/**
 * Removes from database record with given *key*.
 * This function is guaranteed to be implemented by all engines.
 *
 * @param[in] key record's key to query for, to be removed
 *
 * @return pmem::kv::status
 */
inline status db::remove(string_view key) noexcept
{
	return static_cast<status>(pmemkv_remove(this->_db, key.data(), key.size()));
}

/**
 * Returns a human readable string describing the last error.
 */
static inline std::string errormsg()
{
	return std::string(pmemkv_errormsg());
}

} /* namespace kv */
} /* namespace pmem */

#endif /* LIBPMEMKV_HPP */
