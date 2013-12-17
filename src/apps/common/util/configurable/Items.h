/*
 * Items.h
 *
 *  Created on: Sep 6, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <cstdlib>


class SettableItem : public ConfigurableTypes
{
public:
	SettableItem(
			bool configurable,
			bool interactive)
	: configurable(configurable),
	  interactive(interactive){}

	virtual ~SettableItem(){}
	virtual bool set(const std::string &specifier) = 0;
	virtual std::string get() = 0;

	using ConfigurableTypes::get;
	using ConfigurableTypes::set;

	bool configurable;	// antonym	= readonly
	bool interactive;	//			= interactive interaction
};


template<typename ValueT>
class ValueItem : public SettableItem
{
public:
	ValueItem(ValueT &var, bool interactive)
	: SettableItem(true, interactive), var(&var){}

	ValueItem(ValueT &var, const ValueT &initial, bool interactive)
	: SettableItem(true, interactive), var(&var) {*(this->var) = initial;};

	virtual bool set(const std::string &specifier)
	{
		return false;
	}

	virtual std::string get()
	{
		return "Error: no string representation";
	}

	virtual bool set(ValueT value)
	{
		*var = value;
		return true;
	}

	virtual bool get(ValueT &value)
	{
		value = *var;
		return true;
	}

protected:
	ValueT *var;
};


template<>
inline bool ValueItem<std::string>::set(const std::string &specifier)
{
	*var = specifier;
	return true;
}


template<>
inline std::string ValueItem<std::string>::get()
{
	return *var;
}

/* No need for functions both for double and float if using
 * C++11, std::is_floating_point etc.
 */
template<>
inline bool ValueItem<double>::set(const std::string &specifier)
{
	*var = std::atof(specifier.c_str());
	return true;
}


template<>
inline bool ValueItem<float>::set(const std::string &specifier)
{
	*var = (float)std::atof(specifier.c_str());
	return true;
}


template<>
inline bool ValueItem<int>::set(const std::string &specifier)
{
	*var = std::atoi(specifier.c_str());
	return true;
}


template<>
inline bool ValueItem<bool>::set(const std::string &specifier)
{
	if(specifier == "true" || specifier == "on" || specifier == "1")
		*var = true;
	else if(specifier == "false" || specifier == "off" || specifier == "0")
		*var = false;
	else
		return false;
	return true;
}


template<>
inline std::string ValueItem<double>::get()
{
	std::stringstream sstr;
	sstr << *var;
	return sstr.str();
}


template<>
inline std::string ValueItem<float>::get()
{
	std::stringstream sstr;
	sstr << *var;
	return sstr.str();
}

template<>
inline std::string ValueItem<int>::get()
{
	std::stringstream sstr;
	sstr << *var;
	return sstr.str();
}


template<>
inline std::string ValueItem<bool>::get()
{
	if(*var) return "true";
	else return "false";
}


template<typename ValueT>
class ReadonlyValueItem : public ValueItem<ValueT>
{
public:
	ReadonlyValueItem(ValueT &var, bool interactive)
	: ValueItem<ValueT>(var, interactive){this->configurable = false;}

	bool set(const std::string &specifier)
	{
		return false;
	}

	bool set(ValueT value)
	{
		return false;
	}
};


template<typename ValueT>
class ArrayValueItem : public SettableItem
{
public:
	ArrayValueItem(ValueT ** array, int wrap, const int &num_elements, bool interactive)
	: SettableItem(false, interactive), array_var(array), array_copy(NULL),
	  wrap(wrap), num_elements_copy(num_elements), num_elements(&num_elements_copy){}

	ArrayValueItem(ValueT ** array, int wrap, int &num_elements, bool interactive)
	: SettableItem(false, interactive), array_var(array), array_copy(NULL),
	  wrap(wrap), num_elements_copy(0), num_elements(&num_elements){}

	ArrayValueItem(ValueT *array, int wrap, const int &num_elements, bool interactive)
	: SettableItem(false, interactive), array_var(&array_copy), array_copy(array),
	  wrap(wrap), num_elements_copy(num_elements), num_elements(&num_elements_copy){}

	ArrayValueItem(ValueT *array, int wrap, int &num_elements, bool interactive)
	: SettableItem(false, interactive), array_var(&array_copy), array_copy(array),
	  wrap(wrap), num_elements_copy(0), num_elements(&num_elements){}

	bool set(const std::string &specifier)
	{
		return false;
	}

	std::string get()
	{
		std::stringstream str;
		str << std::setiosflags(std::ios::fixed) << std::setprecision(4);

		int cellw0 = 5;
		int cellw = 15;

		if(wrap > 1)
		{
			str << std::setw(cellw0) << ' ';

			for(int col = 0; col < wrap; ++col)
				str << std::setw(cellw) << col;
		}

		int printed = 0;
		int rows = 0;
		while(printed < *num_elements)
		{
			if(printed % wrap == 0)
			{
				str << std::endl;
				str << std::setw(cellw0) << rows;
				++rows;
			}

			str << std::setw(cellw) << (*array_var)[printed];

			++printed;
		}

		return str.str();
	}

	bool get(ValueT * &input_array)
	{
		input_array = *array_var;
		return true;
	}

	bool set(ValueT * &new_array)
	{
		*array_var = new_array;
		return true;
	}

protected:
	ValueT **array_var;
	ValueT *array_copy;
	int wrap;
	const int num_elements_copy;
	const int *num_elements;
};


template<typename ValueT>
class ReadonlyArrayValueItem : public ArrayValueItem<ValueT>
{
public:
	ReadonlyArrayValueItem(
			ValueT *array,
			int wrap,
			const int &num_elements,
			bool interactive)
	: ArrayValueItem<ValueT>(array, wrap, num_elements, interactive){}

	ReadonlyArrayValueItem(
			ValueT *array,
			int wrap,
			int &num_elements,
			bool interactive)
	: ArrayValueItem<ValueT>(array, wrap, num_elements, interactive){}

	bool set(ValueT * &value)
	{
		return false;
	}

	bool get(ValueT * &value)
	{
		return false;
	}

	bool get(const ValueT * &value)
	{
		value = *(this->array_var);
		return true;
	}
};


template<typename ValueT>
class ValueSetItem : public SettableItem
{
public:
	ValueSetItem(
			ValueT &var,
			const std::map<const std::string, ValueT> &value_set,
			const std::string &initial,
			bool interactive)
	: SettableItem(true, interactive),
	  var(&var),
	  value_set(&value_set)
	{set_initial(initial);}

	void set_initial(const std::string &initial)
	{
		typename std::map<const std::string, ValueT>::const_iterator it;
		it = this->value_set->find(initial);
		if(it == this->value_set->end())
		{
			it = this->value_set->begin();
			std::cerr << "Warning: unknown specifier in initialization: " <<
					initial << ", changed to: " << it->first << std::endl;
		}

		*(this->var) = it->second;
		current = it;
	}

	bool set(const std::string &specifier)
	{
		typename std::map<const std::string, ValueT>::const_iterator it;
		it = value_set->find(specifier);
		if(it != value_set->end())
		{
			*var = it->second;
			current = it;
			return true;
		}
		return false;
	}

	std::string get()
	{
		std::string available(" ( ");
		typename std::map<const std::string, ValueT>::const_iterator it;
		for(it = value_set->begin(); it != value_set->end(); ++it)
		{
			if(it != current) available += "'" + it->first + "' ";
		}
		available += (")");
		return current->first + available;
	}

private:
	ValueT *var;
	typename std::map<const std::string, ValueT>::const_iterator current;
	const std::map<const std::string, ValueT> *value_set;
};
