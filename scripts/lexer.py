# ------------------------------------------------------------
# lexer.py
#
# tokenizer for a set of commands given by the user through
# a speech recognition interface
# ------------------------------------------------------------
import ply.lex as lex

# List of token names.   This is always required
tokens = (
   'GO',
   'TO',
   'FOLLOW',
   'ME',
   'FOR',
   'SECONDS',
   'GRASP',
   'OBJECT',
   'ASK',
   'AS',
   'ENACT',
   'POSE',
   'MOVE',
   'HAND',
   'REQUEST',
   'GRASPING',
   'MOVEMENT',
   'PRIMITIVE',
   'RECOGNIZE',
   'EXECUTE',
   'RECORD',
   'YOU',
   'ARE',
   'IN',
   'ADJUST',
   'COURSE',
   'BY',
   'AND',
   'THEN',
   'THIS',
   'BACKWARD',
   'FORWARD',
   'UP',
   'MOVING',
   'LEFT',
   'DOWN',
   'RIGHT',
)

# Regular expression rules for simple tokens
t_PLUS    = r'\+'
t_MINUS   = r'-'
t_TIMES   = r'\*'
t_DIVIDE  = r'/'
t_LPAREN  = r'\('
t_RPAREN  = r'\)'
t_ID      = r'[A-Za-z]+'

# A regular expression rule with some action code
def t_NUMBER(t):
    r'\d+'
    t.value = int(t.value)    
    return t

# Define a rule so we can track line numbers
def t_newline(t):
    r'\n+'
    t.lexer.lineno += len(t.value)

# A string containing ignored characters (spaces and tabs)
t_ignore  = ' \t'

# Error handling rule
def t_error(t):
    print("Illegal character '%s'" % t.value[0])
    t.lexer.skip(1)

# Build the lexer
lexer = lex.lex()