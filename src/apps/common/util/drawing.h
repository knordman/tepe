/*
 * drawing.h
 *
 *  Created on: Aug 22, 2012
 *      Author: Kristian Nordman <knordman@kth.se>
 */


#pragma once

#include "drawstuff/drawstuff.h"

#ifdef dDOUBLE
#define dsDrawLine dsDrawLineD
#define dsDrawSphere dsDrawSphereD
#endif


inline void draw_hinges(BasicWorld *sw)
{
	for(int h = 0; h < (TP_HINGES); ++h)
	{
		if(!sw->draw_hinges[h]) continue;

		for(int i = 0; i < 2; ++i)
		{
			tp_vec3 rg;
			get_vec3(pos(sw->mem, _Jm(sw->mem, 5*h, i)), rg);

			tp_mtx33 _R;
			get_mtx33(R(sw->mem, _Jm(sw->mem, 5*h, i)), _R);

			// Draw rl 0 + 1 in blue
			dsSetColor(0.0, 0.0, 1.0);
			tp_vec3 rl;
			get_vec3(hanchor(sw->mem, h, i), rl);
			mult_to_mtx33_vec3(_R, rl);
			add_to_vec3(rl, rg, 1.0);
			dsDrawLine(rg, rl);

			if(i == 0)
			{
				// Draw axis 0 + 1 in red (axis is in world coords)
				dsSetColor(1.0, 0.0, 0.0);
				tp_vec3 ra;
				get_vec3(haxis(sw->mem, h), ra);
				mult_to_mtx33_vec3(_R, ra);
				add_to_vec3(ra, rl, 1.0);
				dsDrawLine(rl, ra);

				// Draw tangent0 0 + 1 in black
				tp_vec3 t;
				dsSetColor(0.0, 0.0, 0.0);
				get_vec3(ht0(sw->mem, h), t);
				mult_to_mtx33_vec3(_R, t);
				add_to_vec3(t, rl, 1.0);
				dsDrawLine(rl, t);

				// Draw tangent1 0 + 1 in black
				dsSetColor(0.0, 0.0, 0.0);
				get_vec3(ht1(sw->mem, h), t);
				mult_to_mtx33_vec3(_R, t);
				add_to_vec3(t, rl, 1.0);
				dsDrawLine(rl, t);
			}
			else
			{
				// Draw axis 0 + 1 in red (axis is in world coords)
				dsSetColor(1.0, 1.0, 0.0);
				tp_vec3 ra;
				get_vec3(haxis_num(sw->mem, h, 1), ra);
				mult_to_mtx33_vec3(_R, ra);
				add_to_vec3(ra, rl, 1.0);
				dsDrawLine(rl, ra);
			}
		}
	}
}

inline void draw_body_refs(BasicWorld *sw)
{
	for(int b = 0; b < (TP_BODIES); ++b)
	{
		if(!sw->draw_body_reframes[b]) continue;

		tp_vec3 rg;
		get_vec3(pos(sw->mem, b), rg);

		tp_mtx33 _R;
		get_mtx33(R(sw->mem, b), _R);

		// -----------------------
		tp_vec3 z = {0, 0, 0.5}, y = {0, 0.5, 0.0}, x = {0.5, 0, 0.0};
		mult_to_mtx33_vec3(_R, x);
		mult_to_mtx33_vec3(_R, y);
		mult_to_mtx33_vec3(_R, z);
		add_to_vec3(x, rg, 1.0);
		add_to_vec3(y, rg, 1.0);
		add_to_vec3(z, rg, 1.0);
		dsSetColor(1.0, 0.0, 0.0);
		dsDrawLine(rg, x);
		dsSetColor(0.0, 0.0, 1.0);
		dsDrawLine(rg, y);
		dsSetColor(0.0, 0.0, 0.0);
		dsDrawLine(rg, z);
		// -----------------------
	}
}

inline void draw_contacts(BasicWorld *sw)
{
	if(!sw->draw_contacts) return;

#ifdef TP_DEBUG

	for(int c = 0; c < sw->num_contacts; ++c)
	{
		tp_vec3 _cpo, _cno;
		get_vec3(cpo(sw->mem, c), _cpo);
		get_vec3(cno(sw->mem, c), _cno);

		tp_mtx33 R;
		set_to_ident_mtx33(R);
		dsSetColor(1.0,0.0,0.0);
		dsDrawSphere(_cpo,R,0.02);

		add_to_vec3(_cno, _cpo, 1.0);
		dsSetColor(0.0,1.0,0.0);
		dsDrawLine(_cpo, _cno);
	}

	for(int contact_body = 0; contact_body < sw->num_contacts/TP_CONTACTS_ON_FOOT; ++contact_body)
	{
		index_t foot_body = _cbdy(sw->mem, contact_body);
		tp_vec3 rg;
		get_vec3(pos(sw->mem, foot_body), rg);

		tp_vec3 _cpl0, _cpl1;
		get_vec3(cpl0(sw->mem, contact_body), _cpl0);
		get_vec3(cpl1(sw->mem, contact_body), _cpl1);

		add_to_vec3(_cpl0, rg, 1.0);
		add_to_vec3(_cpl1, rg, 1.0);

		dsSetColor(0.0,0.0,0.0);
		dsDrawLine(rg, _cpl0);
		dsDrawLine(rg, _cpl1);
	}
#endif
}
