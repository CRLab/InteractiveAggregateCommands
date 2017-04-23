# ------------------------------------------------------------
# lexer.py
#
# tokenizer for a set of commands given by the user through
# a speech recognition interface
# ------------------------------------------------------------
import ply.lex as lex
import ply.yacc as yacc
from ast import *
tokens = Lexer.tokens

class CommandLexer(object):
    # List of token names.   This is always required
    tokens = (
       'AS',
       'EXECUTE',
       'RECORD',
       'ID',
    )

    #Definitions of tokens go in functions to increase precidence
    def t_AS(self, t): r'as'; return t
    def t_EXECUTE(self, t): r'execute'; return t
    def t_RECORD(self, t): r'record'; return t

    def t_ID(self,t): r'[a-zA-Z][a-zA-Z0-9\s]*[a-zA-Z]'; return t

    # Define a rule so we can track line numbers
    def t_newline(self,t):
        r'\n+'
        t.lexer.lineno += len(t.value)

    # A string containing ignored characters (spaces and tabs)
    t_ignore  = ' \t'

    # Error handling rule
    def t_error(self,t):
        print("Illegal character '%s'" % t.value[0])
        t.lexer.skip(1)

    # Build the lexer
    def build(self,**kwargs):
        self.lexer = lex.lex(module=self, **kwargs)
    
    # Test it output
    def test(self,data):
        self.lexer.input(data)
        while True:
             tok = self.lexer.token()
             if not tok: 
                 break
             print(tok)

tokens = CommandLexer.tokens
############################## COMMANDS ##############################
#command : 
    #EXECUTE <task list>
    #RECORD <task list> as ID

def p_command_execute(p):
    'command : EXECUTE ID'
    p[0] = UnprocessedExecute(p[2])

def p_command_record(p):
    'command : RECORD ID AS ID'
    p[0] = UnprocessedRecord(p[2], p[4])

# Error rule for syntax errors
def p_error(p):
    print("Syntax error in input!")

def p_top_level(p):
    'start : command'
    p[0] = p[1]

# Build the parser
lexer_instance = CommandLexer()
lexer_instance.build()           # Build the lexer
command_parser = yacc.yacc(start='start', lexer=lexer_instance.lexer)