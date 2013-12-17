%error-verbose
%parse-param {World *world}

%code top {
#include <iostream>
#include <string>
#include <exception>
#include <algorithm>
}

%code requires {
#include <apps/common/util/world/World.h>
#include <vector>
int yylex();
void flex_reset_buffer();
}

%code {
void yyerror(World *world, const char *msg);
inline void free_strings(std::string *s1, std::string *s2 = NULL, std::string *s3 = NULL, std::string *s4 = NULL);
}

%union values {
	int integer_val;
	real_t float_val;
	std::string * string_val;
	std::vector<std::string> * context_chain;
}

%token <string_val> INTEGER <string_val> FLOAT <string_val> STRING
%token SET PRINT NEXT ACTIVATE CREATE END RSQUAREBRACKET LSQUAREBRACKET
%type <context_chain> contextlist
%type <string_val> value

%destructor { delete $$; } INTEGER
%destructor { delete $$; } FLOAT
%destructor { delete $$; } STRING

%%
cmds:	/* empty */
		| cmds cmd
		;
cmd:	set
		| print
		| activate
		| create
		;
set:				SET STRING value END {
						world->set_property(*($2), *($3));						
						free_strings($2, $3);
					}
					|
					SET NEXT contextlist STRING value END {
						std::reverse(($3)->begin(), ($3)->end());
						world->delegate_set_property(*($3), *($4), *($5));
						delete $3;
						free_strings($4, $5);
					}
/*					|
					SET STRING LSQUAREBRACKET INTEGER RSQUAREBRACKET value END {
						int index = std::atoi(($4)->c_str());
						
						std::string num_elements_name = "num " + *($2);
					
						int num_elements;
						if(world->get_property_by_value(num_elements_name.c_str(), num_elements))
						{
							if(index < num_elements)
							{
								real_t *array;
								if(world->get_property_by_value(*($2), array))
								{
									array[index] = std::atof(($6)->c_str());
								}
								else
									std::cerr << "Error: unable to set (by float) " << *($2) << std::endl;
							}
							else
								std::cerr << "Error: index (" << *($4) << ")out of bounds, array size is " << num_elements << std::endl;
						}
						
						free_strings($2, $4, $6);
					}
					|
					SET NEXT contextlist STRING LSQUAREBRACKET INTEGER RSQUAREBRACKET part FLOAT FLOAT FLOAT END {
						std::reverse(($3)->begin(), ($3)->end());
						std::vector<std::string> chain_copy = *($3);
						
						int index = std::atoi(($6)->c_str());
						
						int num_points;
						if(world->delegate_get_property_by_value(chain_copy, "num points", num_points))
						{
							if(index > num_points - 1)
							{
								std::cerr << "Error: index out of bounds, array size is " << num_points << std::endl;
							}
							else
							{
								PathPoint *array;
								if(world->delegate_get_property_by_value(*($3), *($4), array))
								{
									switch($8)
									{
										case PATH_POINT_CONTROL:
											array[index].control[0] = std::atof(($9)->c_str());
											array[index].control[1] = std::atof(($10)->c_str());
											array[index].control[2] = std::atof(($11)->c_str());
										break;
										case PATH_POINT_POSITION:
										default:
											array[index].pos[0] = std::atof(($9)->c_str());
											array[index].pos[1] = std::atof(($10)->c_str());
											array[index].pos[2] = std::atof(($11)->c_str());
										break;
									}
								}
							}
						}
						else
						{
							std::cerr << "Error: the specified context does not seem to have a pathpoint array" << std::endl;
						}
												
						delete $3;
						free_strings($4, $6);
						free_strings($9, $10, $11);
					}*/
					;
value:				STRING
					|
					FLOAT
					|
					INTEGER
					;
activate:			ACTIVATE STRING END {
						world->activate_subcontext(*($2));
						free_strings($2);
					}
					|
					ACTIVATE STRING STRING END {
						world->activate_subcontext(*($2), *($3));
						free_strings($2, $3);
					}
					|
					ACTIVATE NEXT contextlist STRING END {
						std::reverse(($3)->begin(), ($3)->end());
						world->delegate_activate_subcontext(*($3), *($4), *($4));
						delete $3;
						free_strings($4);
					}
					|
					ACTIVATE NEXT contextlist STRING STRING END {
						std::reverse(($3)->begin(), ($3)->end());
						world->delegate_activate_subcontext(*($3), *($4), *($5));
						delete $3;
						free_strings($4, $5);
					}
					;
create:				CREATE STRING END {
						std::stringstream errors;

						if(!world->create_subcontext(*($2), errors))
						{
							std::cerr << errors.str();
							std::cerr << "Error: could not create '" << *($2) << "'" << std::endl;
						}

						free_strings($2);
					}
					|
					CREATE NEXT contextlist STRING END {
						std::stringstream errors;
						std::reverse(($3)->begin(), ($3)->end());

						if(!world->delegate_create_subcontext(*($3), *($4), errors))
						{
							std::cerr << errors.str();
							std::cerr << "Error: could not create '" << *($4) << "'" << std::endl;
						}

						delete $3;
						free_strings($4);
					}
					;
print:				PRINT END {
						world->print_properties();
					}
					|
					PRINT STRING END {
						const std::string value = world->get_property(*($2));
						std::cout << value << std::endl;						
						free_strings($2);
					}
					|
					PRINT NEXT contextlist STRING END {
						std::reverse(($3)->begin(), ($3)->end());
						const std::string value = world->delegate_get_property(*($3), *($4));
						std::cout << value << std::endl;
						delete $3;
						free_strings($4);
					}
					|
					PRINT NEXT contextlist END {
						std::reverse(($3)->begin(), ($3)->end());
						world->delegate_print_properties(*($3));			
						delete $3;
					}
					;
					
contextlist:		STRING { 
						$$ = new std::vector<std::string>(); 
						$$->push_back(*($1));
					}
					|
					contextlist NEXT STRING {
						$1->push_back(*($3));
					}
					;
%%

void yyerror(World *world, const char *msg)
{
	std::cerr << "Error: " << msg << '.' << std::endl;
	flex_reset_buffer();
}

inline void free_strings(std::string *s1, std::string *s2, std::string *s3, std::string *s4)
{
	if(s1) delete s1;
	if(s2) delete s2;
	if(s3) delete s3;
	if(s4) delete s4;
}
