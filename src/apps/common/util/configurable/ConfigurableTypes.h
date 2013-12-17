/*
 * ConfigurableTypes.h
 *
 *  Created on: Sep 5, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include <tp/types/default.h>

class ConfigurableTypes
{
public:
	virtual ~ConfigurableTypes(){}

	virtual bool set(real_t value){return false;}
	virtual bool set(int value){return false;}
	virtual bool set(bool value){return false;}

	virtual bool get(real_t &value){return false;}
	virtual bool get(real_t * &value){return false;}
	virtual bool get(const real_t * &value){return false;}
	virtual bool get(int &value){return false;}
	virtual bool get(unsigned &value){return false;}
	virtual bool get(bool &value){return false;}
};

