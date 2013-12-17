/*
 * jprinter.h
 *
 *  Created on: Aug 24, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

inline void printJ(struct mem_t *mem)
{
	std::cout << std::setw(3) << ' ';
	for(int col = 0; col < TP_BODIES; ++col)
		std::cout << std::setw(4) << col;
	std::cout << std::endl;

	for(int s = 0; s < TP_CONSTRAINTS; ++s)
	{
		index_t b0 = _Jm(mem, s, 0);
		index_t b1 = _Jm(mem, s, 1);

		std::cout << std::setw(3) << s;

		for(int col = 0; col < TP_BODIES; ++col)
		{
			if(col == b0 && col == b1)
				std::cout << std::right << std::setw(4) << '-';
			else if(col == b0)
				std::cout << std::right << std::setw(4) << 'o';
			else if(col == b1)
				std::cout << std::right << std::setw(4) << 'x';
			else
				std::cout << std::right << std::setw(4) << ' ';
		}

		std::cout << std::endl;
	}
}

