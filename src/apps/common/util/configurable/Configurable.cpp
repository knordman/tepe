/*
 * Context.h
 *
 *  Created on: Aug 15, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */

#include "Configurable.h"
#include <algorithm>


bool Configurable::set_property(const std::string &property, const std::string &specifier)
{
	std::map<const std::string, SettableItem *>::iterator found;

	found = properties.find(property);
	if(found != properties.end())
	{
		if(!found->second->set(specifier))
			std::cerr << "Warning: unable to set property '" <<
			property << "' of '" << name << "' to '" << specifier << "'" << std::endl;
		else
			return true;
	}
	else
	{
		std::cerr << "Warning: '" << name << "' does not have property '" << property << "'" << std::endl;
	}

	return false;
}


bool Configurable::delegate_set_property(std::vector<std::string> &chain,
		const std::string &property, const std::string &specifier)
{
	if(chain.size() == 0)
	{
		// We are the chosen ones (perhaps)
		return set_property(property, specifier);
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
			return found->second->delegate_set_property(chain, property, specifier);
		}
		else
		{
			std::cerr << "Error: '" << name << "' does not have context '" << context << "'" << std::endl;
		}
	}

	return false;
}


std::string Configurable::delegate_get_property(std::vector<std::string> &chain, const std::string &property)
{
	if(chain.size() == 0)
	{
		// We are the chosen ones (perhaps)
		return get_property(property);
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
			return found->second->delegate_get_property(chain, property);
		}
		else
		{
			std::cerr << "Error: '" << name << "' does not have context '" << context << "'" << std::endl;
		}
	}

	return "";
}


Configurable * Configurable::delegate_activate_subcontext(
		std::vector<std::string> &chain,
		const std::string &key,
		const std::string &context_name)
{
	if(chain.size() == 0)
	{
		// We are the chosen ones (perhaps)
		return activate_subcontext(key, context_name);
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
			return found->second->delegate_activate_subcontext(chain, key, context_name);
		}
		else
		{
			std::cerr << "Error: '" << name << "' does not have context '" << context << "'" << std::endl;
		}
	}

	return NULL;
}


bool Configurable::delegate_create_subcontext(
		std::vector<std::string> &chain,
		const std::string &key,
		std::stringstream &errors)
{
	if(chain.size() == 0)
	{
		// We are the chosen ones (perhaps)
		return create_subcontext(key, errors);
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
			return found->second->delegate_create_subcontext(chain, key, errors);
		}
		else
		{
			std::cerr << "Error: '" << name << "' does not have context '" << context << "'" << std::endl;
		}
	}

	return false;
}


Configurable * Configurable::activate_subcontext(const std::string &key, const std::string &context_name)
{
	BaseAllocator *allocator = NULL;

	std::map<const std::string, std::map<const std::string, BaseAllocator *> >::iterator found_available;
	found_available = available_sub_contexts.find(key);

	if(found_available != available_sub_contexts.end())
	{
		std::map<const std::string, BaseAllocator *>::iterator found_context;
		found_context = found_available->second.find(context_name);

		if(found_context != found_available->second.end())
		{
			allocator = found_context->second;
		}
		else
		{
			std::cerr << "Error: '" << context_name << "' is not a supported '" << key << "'" << std::endl;
			return NULL;
		}
	}
	else
	{
		std::cerr << "Error: '" << name << "' does not have a '" << key << "' context" << std::endl;
		return NULL;
	}

	std::map<const std::string, Configurable *>::iterator found_active;

	Configurable *context = (*allocator)();

	found_active = active_sub_contexts.find(key);
	if(found_active != active_sub_contexts.end())
	{
		delete found_active->second;

		found_active->second = context;
	}
	else
	{
		active_sub_contexts.insert(std::pair<const std::string, Configurable *>(key, context));
	}

	return context;
}


bool Configurable::create_subcontext(const std::string &key, std::stringstream &errors)
{
	std::map<const std::string, Configurable *>::iterator found_active;

	found_active = active_sub_contexts.find(key);
	if(found_active != active_sub_contexts.end())
	{
		if(!found_active->second->created)
			return found_active->second->create(errors);
		else
			errors << "Error: '" << key << "' already created, to reset '"
					<< key << "', re-activate it" << std::endl;
	}

	return false;
}


void Configurable::clear_active_subcontexts()
{
	std::map<const std::string, Configurable *>::iterator active_contexts_iterator;
	for(active_contexts_iterator = active_sub_contexts.begin();
			active_contexts_iterator != active_sub_contexts.end();
			++active_contexts_iterator)
	{
		delete active_contexts_iterator->second;
	}
	active_sub_contexts.clear();
}



Configurable::~Configurable()
{
	std::map<const std::string, SettableItem *>::iterator prop_iterator;

	for(prop_iterator = properties.begin(); prop_iterator != properties.end(); ++prop_iterator)
	{
		delete prop_iterator->second;
	}

	clear_active_subcontexts();

	std::map<const std::string, std::map<const std::string, BaseAllocator *> >::iterator keys_with_contexts_iterator;
	for(keys_with_contexts_iterator = available_sub_contexts.begin();
			keys_with_contexts_iterator != available_sub_contexts.end();
			++keys_with_contexts_iterator)
	{
		std::map<const std::string, BaseAllocator *>::iterator available_contexts_iterator;
		for(available_contexts_iterator = keys_with_contexts_iterator->second.begin();
				available_contexts_iterator != keys_with_contexts_iterator->second.end();
				++available_contexts_iterator)
			delete available_contexts_iterator->second;
	}
}


void Configurable::delegate_print_properties(std::vector<std::string> &chain)
{
	if(chain.size() == 0)
	{
		// We are the chosen ones
		print_properties();
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
			found->second->delegate_print_properties(chain);
		}
		else
		{
			std::cerr << "Error: '" << name << "' does not have context '" << context << "'" << std::endl;
		}
	}
}


static void print_columns(
		const std::vector<std::string> &column0,
		const std::string &column0_name,
		const std::vector<std::string> &column1,
		const std::string &column1_name,
		int cell_width)
{
	int rows = std::max(column0.size(), column1.size());

	std::vector<std::string>::const_iterator c0it = column0.begin();
	std::vector<std::string>::const_iterator c1it = column1.begin();

	std::cout << std::left << std::setfill('-') << std::setw(cell_width) << column0_name + ' ';
	std::cout << std::left << std::setfill('-') << std::setw(cell_width) << ' ' + column1_name + ' ' << std::endl;
	std::cout << std::setfill(' ');

	for(int row = 0; row < rows; ++row)
	{
		if(c0it != column0.end())
		{
			std::cout << std::setw(cell_width) << *c0it;
			++c0it;
		}
		else std::cout << std::setw(cell_width) << "";

		if(c1it != column1.end())
		{
			std::cout << std::setw(cell_width) << *c1it;
			++c1it;
		}
		else std::cout << std::setw(cell_width) << "";

		std::cout << std::endl;
	}
}


void Configurable::print_properties()
{
	int cell_width = 40;

	std::cout << "Properties of '" << name << std::endl << std::endl;

	std::vector<std::string> interactive;
	std::vector<std::string> readonly;
	std::vector<std::string> create;
	std::vector<std::string> internal;

	for(std::map<const std::string, SettableItem *>::iterator it = properties.begin();
			it != properties.end();
			++it)
	{
		if(it->second->configurable)
		{
			if(it->second->interactive)
				interactive.push_back("'" + it->first + "'");
			else
				create.push_back("'" + it->first + "'");
		}
		else
		{
			if(it->second->interactive)
				readonly.push_back("'" + it->first + "'");
			else
				internal.push_back("'" + it->first + "'");
		}
	}

	print_columns(interactive, "Interactive", readonly, "Readonly", cell_width);
	std::cout << std::endl;

	if(create.size() > 0 || internal.size() > 0)
	{
		print_columns(create, "Set prior to creating", internal, "Internal", cell_width);
		std::cout << std::endl;
	}

	std::vector<std::string> active;
	std::vector<std::string> notactive;

	for(std::map<const std::string, std::map<const std::string, BaseAllocator *> >::iterator
			it = available_sub_contexts.begin();
			it != available_sub_contexts.end();
			++it)
	{
		std::map<const std::string, Configurable *>::iterator found
			= active_sub_contexts.find(it->first);

		std::vector<std::string> *pushed_to;

		if(found != active_sub_contexts.end())
		{
			std::string label;
			if(found->second->created)
				label += "(C) ";
			else
				label += "(A) ";

			std::string post_label;
			if(it->second.size() > 1)
				post_label = "('" + found->second->name + "')";

			active.push_back(label + "'" + found->first + "' " + post_label);
			pushed_to = &active;
		}
		else
		{
			notactive.push_back("'" + it->first + "'");
			pushed_to = &notactive;
		}

		if(it->second.size() > 1)
		{
			// A context with alternatives
			for(std::map<const std::string, BaseAllocator *>::iterator
					iter = it->second.begin();
					iter != it->second.end();
					++iter)
			{
				pushed_to->push_back(" - '" + iter->first + "'");
			}
		}
	}

	if(active.size() > 0 || notactive.size() > 0)
	{
		print_columns(active, "(A)ctive + (C)reated subcontexts", notactive, "Inactive subcontexts", cell_width);
		std::cout << std::endl;
	}

	std::cout << std::setfill('-') << std::setw(2*cell_width) << "" << std::endl;
}
