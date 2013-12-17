%option noyywrap nounput
%{
#include "parser.hpp"
static bool quote_mode = false;
void toggle_quote_mode();
%}

%x quote_string

%%
set									return SET;
print								return PRINT;
activate							return ACTIVATE;
create								return CREATE;
;									return END;
\>									return NEXT;
\[									return LSQUAREBRACKET;
\]									return RSQUAREBRACKET;

\'									toggle_quote_mode();

[0-9]+								yylval.string_val = new std::string(yytext); return INTEGER;
[-]*[[:digit:]]*"."[[:digit:]]+		yylval.string_val = new std::string(yytext); return FLOAT;
[a-zA-Z0-9]+						yylval.string_val = new std::string(yytext); return STRING;

<quote_string>[^\']+				BEGIN(INITIAL); yylval.string_val = new std::string(yytext); return STRING;

.|\n								/* Ignore */				
%%

void flex_reset_buffer()
{
	yy_flush_buffer(YY_CURRENT_BUFFER);
}

void toggle_quote_mode()
{
	if(quote_mode) 
		BEGIN(INITIAL);
	else 
		BEGIN(quote_string);
		
	quote_mode = !(quote_mode);
}
