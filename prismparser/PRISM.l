D			[0-9]
L			[a-zA-Z_]
H			[a-fA-F0-9]
E			[Ee][+-]?{D}+
FS			(f|F|l|L)
IS			(u|U|l|L)*

%{
#include "AST.h"
#include "PRISM.tab.hpp"
#include "PRISMParser.h"

extern prismparser_ast::Substitution formulas;

extern void PRISMerror(char *s);
extern void PRISMerror(const std::string&);

namespace prismparser {
  extern int line_number;
  extern int propmode;
}

namespace prismparser {
  void emit(int current_token = BOGY);
  void comment();
  int token;
  int check_type();

  extern prismparser_ast::Substitution formulas;
}
using namespace prismparser;

%}

%s line_directive
%%

"/*"                    { emit(); comment();         }
[ \t]+                  { emit();                    }
\/\/[^\n]*              { emit();                    }
"module"                { emit(MODULE);    return token; }
"endmodule"             { emit(ENDMODULE); return token; }
"const"                 { emit(CONSTANT);  return token; }
"global"                { emit(GLOBAL);    return token; }
"param"                 { emit(PARAM);     return token; }
"formula"               { emit(FORMULA);   return token; }
"label"                 { emit(LABEL);     return token; }
"var"                   { emit(VAR);       return token; }
"init"                  { emit(INIT);      return token; }
"endinit"               { emit(ENDINIT);   return token; }
"invar"                 { emit(INVAR);     return token; }
"endinvar"              { emit(ENDINVAR);  return token; }
"pred"                  { emit(PRED);      return token; }
"float"                 { emit(FLOAT);     return token; }
"int"                   { emit(INTEGER);   return token; }
"bool"                  { emit(BOOL);      return token; }
"double"                { emit(DOUBLE);    return token; }
"true"                  { emit(TRUE);      return token; }
"false"                 { emit(FALSE);     return token; }
"nondeterministic"      { emit(NONDET);    return token; }
"mdp"                   { emit(NONDET);    return token; }
"stochastic"            { emit(STOCH);     return token; }
"dtmc"                  { emit(PROBAB);    return token; }
"ctmc"                  { emit(STOCH);     return token; }
"probabilistic"         { emit(PROBAB);    return token; }
"min"                   { emit(MIN_OP);    return token; }
"max"                   { emit(MAX_OP);    return token; }
"U"                     { emit(UNTIL);     return token; }
"X"                     { emit(NEXT);      return token; }
"F"                     { emit(FINALLY);   return token; }
"C"                     { emit(CUMULATIVE);return token; }
"S"                     { emit(STEADY);    return token; }
"Smin"                  { emit(STEADYMIN); return token; }
"Smax"                  { emit(STEADYMAX); return token; }
"P"                     { emit(PROB);      return token; }
"Pmax"                  { emit(PROBMAX);   return token; }
"Pmin"                  { emit(PROBMIN);   return token; }
"rewards"               { emit(REWARDS);   return token; }
"endrewards"            { emit(ENDREWARDS);return token; }
"I"                     { emit(I_REW);     return token; }
"R"                     { emit(REW);       return token; }
"Rmin"                  { emit(REWMIN);    return token; }
"Rmax"                  { emit(REWMAX);    return token; }
"pow"			{ emit(POW);       return token; }

{L}({L}|{D})* { 
	emit(IDENTIFIER);  
	PRISMlval.string_const = strdup(PRISMtext) ;
	return token; }
	
{D}+    		{ emit(INT_CONSTANT); 
			  PRISMlval.int_const = atoi(PRISMtext);
                          return token; }
".."                    { emit(DOTS);      return token; }
{D}+{E}{FS}?	        { emit(FLOAT_CONSTANT); 
			  PRISMlval.float_const = strdup(PRISMtext);
                          return token; }
{D}*"."[^ "."]{D}+({E})?{FS}?	{ emit(FLOAT_CONSTANT); 
			   PRISMlval.float_const = strdup(PRISMtext);
                           return token; }
{D}+"."[^ "."]{D}*({E})?{FS}?	{ emit(FLOAT_CONSTANT); 
			  PRISMlval.float_const = strdup(PRISMtext);
                          return token; }

"->"			{ emit(DISTR);     return token; }
"&"			{ emit('&');       return token; }
"|"			{ emit('|');       return token; }
"=>"			{ emit(IMPL);      return token; }
"<="			{ emit(LE_OP);     return token; }
">="			{ emit(GE_OP);     return token; }
"=="			{ emit(EQ_OP);     return token; }
"!="			{ emit(NE_OP);     return token; }
";"			{ emit(';');       return token; }
("{"|"<%")		{ emit('{');       return token; }
("}"|"%>")		{ emit('}');       return token; }
","			{ emit(',');       return token; }
":"			{ emit(':');       return token; }
"="			{ emit('=');       return token; }
"("			{ emit('(');       return token; }
")"			{ emit(')');       return token; }
("["|"<:")		{ emit('[');       return token; }
("]"|":>")		{ emit(']');       return token; }
"."			{ emit('.');       return token; }
"!"			{ emit('!');       return token; }
"~"			{ emit('~');       return token; }
"-"			{ emit('-');       return token; }
"+"			{ emit('+');       return token; }
"*"			{ emit('*');       return token; }
"/"			{ emit('/');       return token; }
"%"			{ emit('%');       return token; }
"<"			{ emit('<');       return token; }
">"			{ emit('>');       return token; }
"^"			{ emit('^');       return token; }
"?"			{ emit('?');       return token; }
"\""                    { emit(QUOTE);     return token; }
"\'"                    { emit(PRIMED);    return token; }
"\n"                    { line_number++; if (propmode) {emit('\n'); return '\n';} }
<<EOF>>                 { static int once = 0;
                          if (once || !propmode) {
                            return 0;
                          } else {
                            once++;
                            emit('\n');
                            return '\n';
                          }
                        }

%%

int PRISMwrap() {
  return 1;
}

namespace prismparser {

void comment() {
    char c, c1;
	do {
		while ((c = yyinput()) != '*' && c != 0)
                  ;
		if (c=='\n') ++line_number;
	}
    while ((c1 = yyinput()) != '/' && c != 0);
}


int column = 0;

inline void emit(int current_token) {
        token = current_token;
}

int foo() { return yyunput(42,0),2; }
}
