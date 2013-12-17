/*
 * Configurable.h
 *
 *  Created on: Aug 14, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <map>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include "ConfigurableTypes.h"
#include "Items.h"

/** Base class implementing support for runtime configurable properties.
 *
 * Any class that should be run-time configurable should extend this class, and ad
 *
 * @ingroup framework-core
 */
class Configurable
{
private:
	class BaseAllocator
	{
	public:
		virtual ~BaseAllocator(){};
		virtual Configurable * operator()() = 0;
	};

	template<typename T, typename U>
	class Allocator : public BaseAllocator
	{
	public:
		Allocator(U * &value, Configurable *parent)
		: mem(&value), parent(parent){}
		Configurable * operator()()
		{
			*mem = new T(parent);
			return *mem;
		}
	private:
		U **mem;
		Configurable *parent;
	};

public:
	Configurable(const std::string &name) : name(name), created(false){}

	/** Creates the object. The object is created using the state of its properties,
	 * that state could possible be inconsistent, creation could fail.
	 *
	 * @param errors The stringstream to which any error descriptions should be output.
	 * @return \c true if creation is successful otherwise \c false.
	 */
	virtual bool create(std::stringstream &errors){return false;}

	virtual ~Configurable();

	const std::string name;
	bool created;

	bool set_property(const std::string &property, const std::string &specifier);

	template<typename T>
	bool set_property_by_value(const std::string &property, T &value)
	{
		std::map<const std::string, SettableItem *>::iterator found;

		found = properties.find(property);
		if(found != properties.end())
		{
			if(!found->second->set(value))
				std::cerr << "Warning: unable to set property '" << property << "'" <<
				" of '" << name << "' to '" << value << "'" << std::endl;
			else
				return true;
		}
		else
		{
			std::cerr << "Warning: '"
					<< name << "' does not have property '" << property << "'" << std::endl;
		}

		return false;
	}

	std::string get_property(const std::string &property)
	{
		std::map<const std::string, SettableItem *>::iterator found;

		found = properties.find(property);
		if(found != properties.end())
		{
			return found->second->get();
		}

		std::stringstream output;
		output << "Error: '" << name << "' does not have property '" << property << "'";
		return output.str();
	}

	template<typename T>
	bool get_property_by_value(const std::string &property, T &value)
	{
		std::map<const std::string, SettableItem *>::iterator found;

		found = properties.find(property);
		if(found != properties.end())
		{
			if(!found->second->get(value))
				std::cerr << "Error: unable to retrieve property '" <<
				property << "' of '" << name << "'" << std::endl;
			else
				return true;
		}
		else
		{
			std::cerr << "Error: '"
					<< name << "' does not have property '" << property << "'" << std::endl;
		}

		return false;
	}

	Configurable * activate_subcontext(
			const std::string &key,
			const std::string &context_name);

	Configurable * activate_subcontext(const std::string &key)
	{
		return activate_subcontext(key, key);
	}

	bool create_subcontext(
			const std::string &key,
			std::stringstream &errors);

	void clear_active_subcontexts();

	void print_properties();

	bool delegate_set_property(std::vector<std::string> &chain,
			const std::string &property, const std::string &specifier);

	template<typename T>
	bool delegate_set_property_by_value(std::vector<std::string> &chain,
			const std::string &property, const T &value)
	{
		if(chain.size() == 0)
		{
			// We are the chosen ones (perhaps)
			return set_property_by_value(property, value);
		}
		else
		{
			// Do we have a subcontext of the right name?
			const std::string context(chain.back());

			std::map<const std::string, Configurable *>::iterator found;

			found = active_sub_contexts.find(context);
			if(found != active_sub_contexts.end())
			{
				chain.pop_back();
				return found->second->delegate_set_property_by_value(chain, property, value);
			}
			else
			{
				std::cerr << "Error: '"
						<< name << "' does not have a '" << context << "' context" << std::endl;
			}
		}

		return false;
	}

	std::string delegate_get_property(std::vector<std::string> &chain, const std::string &property);

	template<typename T>
	bool delegate_get_property_by_value(std::vector<std::string> &chain, const std::string &property, T &value)
	{
		if(chain.size() == 0)
		{
			// We are the chosen ones (perhaps)
			return get_property_by_value(property, value);
		}
		else
		{
			// Do we have a subcontext of the right name?
			const std::string context(chain.back());

			std::map<const std::string, Configurable *>::iterator found;

			found = active_sub_contexts.find(context);
			if(found != active_sub_contexts.end())
			{
				chain.pop_back();
				return found->second->delegate_get_property_by_value(chain, property, value);
			}
			else
			{
				std::cerr << "Error: '"
						<< name << "' does not have context '" << context << "'" << std::endl;
			}
		}

		return false;
	}

	void delegate_print_properties(std::vector<std::string> &chain);

	Configurable * delegate_activate_subcontext(
			std::vector<std::string> &chain,
			const std::string &key,
			const std::string &context_name);

	bool delegate_create_subcontext(
			std::vector<std::string> &chain,
			const std::string &key,
			std::stringstream &errors);

protected:
	template<typename ValueT>
	void
	add_property(
			const std::string &key,
			ValueT &var)
	{
		ValueItem<ValueT> *v = new ValueItem<ValueT>(var, false);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_property(
			const std::string &key,
			ValueT &var,
			const ValueT &initial)
	{
		ValueItem<ValueT> *v = new ValueItem<ValueT>(var, initial, false);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_interactive_property(
			const std::string &key,
			ValueT &var)
	{
		ValueItem<ValueT> *v = new ValueItem<ValueT>(var, true);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_interactive_property(
			const std::string &key,
			ValueT &var,
			const ValueT &initial)
	{
		ValueItem<ValueT> *v = new ValueItem<ValueT>(var, initial, true);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}


	template<typename ValueT>
	void
	add_set_property(
			const std::string &key,
			ValueT &var,
			const std::map<const std::string, ValueT> &value_set,
			const std::string &initial)
	{
		ValueSetItem<ValueT> *v = new ValueSetItem<ValueT>(var, value_set, initial, false);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_interactive_set_property(
			const std::string &key,
			ValueT &var,
			std::map<const std::string, ValueT> &value_set,
			const std::string &initial)
	{
		ValueSetItem<ValueT> *v = new ValueSetItem<ValueT>(var, value_set, initial, true);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_array_property(
			const std::string &key,
			ValueT * &var,
			int wrap,
			const int &num_elements)
	{
		ArrayValueItem<ValueT> *v = new ArrayValueItem<ValueT>(&var, wrap, num_elements, false);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_array_property(
			const std::string &key,
			ValueT * &var,
			int wrap,
			int &num_elements)
	{
		ArrayValueItem<ValueT> *v = new ArrayValueItem<ValueT>(&var, wrap, num_elements, false);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_readonly_property(
			const std::string &key,
			ValueT &var)
	{
		ReadonlyValueItem<ValueT> *v = new ReadonlyValueItem<ValueT>(var, false);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_interactive_readonly_property(
			const std::string &key,
			ValueT &var)
	{
		ReadonlyValueItem<ValueT> *v = new ReadonlyValueItem<ValueT>(var, true);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_interactive_readonly_array_property(
			const std::string &key,
			ValueT *var,
			int wrap,
			const int &num_elements)
	{
		ReadonlyArrayValueItem<ValueT> *v = new ReadonlyArrayValueItem<ValueT>(var, wrap, num_elements, true);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename ValueT>
	void
	add_interactive_readonly_array_property(
			const std::string &key,
			ValueT *var,
			int wrap,
			int &num_elements)
	{
		ReadonlyArrayValueItem<ValueT> *v = new ReadonlyArrayValueItem<ValueT>(var, wrap, num_elements, true);
		std::pair<const std::string, SettableItem *> to_insert =
				std::pair<const std::string, SettableItem *>(key, v);
		properties.insert(to_insert);
	}

	template<typename T, typename U>
	void
	add_subcontext(const std::string &key, const std::string &context_name, U * &value)
	{
		typename std::map<const std::string, std::map<const std::string, BaseAllocator *> >::iterator found;

		found = available_sub_contexts.find(key);
		if(found != available_sub_contexts.end())
		{
			// key already exists
			Allocator<T, U> *al = new Allocator<T, U>(value, this);
			found->second.insert(std::pair<const std::string, BaseAllocator *>(context_name, al));
		}
		else
		{
			std::map<const std::string, BaseAllocator *> initial_map;

			Allocator<T, U> *al = new Allocator<T, U>(value, this);
			initial_map.insert(std::pair<const std::string, BaseAllocator *>(context_name, al));

			available_sub_contexts.insert(std::pair<const std::string, std::map<const std::string, BaseAllocator *> >
			(key, initial_map));
		}
	}

	template<typename T>
	void
	add_subcontext(const std::string &key, T * &value)
	{
		add_subcontext<T, T>(key, key, value);
	}

private:
	std::map<const std::string, SettableItem *> properties;
	std::map<const std::string, Configurable *> active_sub_contexts;
	std::map<const std::string, std::map<const std::string, BaseAllocator *> > available_sub_contexts;
};
