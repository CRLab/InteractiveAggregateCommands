# ------------------------------------------------------------
# lexer.py
#
# tokenizer for a set of commands given by the user through
# a speech recognition interface
# ------------------------------------------------------------
import ply.lex as lex

class Lexer(object):
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
       'ID',
       'TIMES',
       'DIVIDE',
       'PLUS',
       'MINUS',
       'RPAREN',
       'LPAREN',
       'LOCATION',
       'NUMBER',
       'PLACE',
       'METERS',
    )

    # Regular expression rules for simple tokens
    t_PLUS    = r'\+'
    t_MINUS   = r'-'
    t_TIMES   = r'\*'
    t_DIVIDE  = r'/'
    t_LPAREN  = r'\('
    t_RPAREN  = r'\)'

    #Definitions of tokens go in functions to increase precidence
    def t_GO(self, t): r'go'; return t
    def t_TO(self, t): r'to'; return t
    def t_FOLLOW(self, t): r'follow'; return t
    def t_ME(self, t): r'me'; return t
    def t_FOR(self, t): r'for'; return t
    def t_SECONDS(self, t): r'seconds'; return t
    def t_GRASP(self, t): r'grasp'; return t
    def t_OBJECT(self, t): r'object'; return t
    def t_ASK(self, t): r'ask'; return t
    def t_AS(self, t): r'as'; return t
    def t_ENACT(self, t): r'enact'; return t
    def t_POSE(self, t): r'pose'; return t
    def t_MOVE(self, t): r'move'; return t
    def t_HAND(self, t): r'hand'; return t
    def t_REQUEST(self, t): r'request'; return t
    def t_GRASPING(self, t): r'grasping'; return t
    def t_MOVEMENT(self, t): r'movement'; return t
    def t_PRIMITIVE(self, t): r'primitive'; return t
    def t_RECOGNIZE(self, t): r'recognize'; return t
    def t_EXECUTE(self, t): r'execute'; return t
    def t_RECORD(self, t): r'record'; return t
    def t_YOU(self, t): r'you'; return t
    def t_ARE(self, t): r'are'; return t
    def t_IN(self, t): r'in'; return t
    def t_ADJUST(self, t): r'adjust'; return t
    def t_COURSE(self, t): r'course'; return t
    def t_BY(self, t): r'by'; return t
    def t_AND(self, t): r'and'; return t
    def t_THEN(self, t): r'then'; return t
    def t_THIS(self, t): r'this'; return t
    def t_BACKWARD(self, t): r'backward'; return t
    def t_FORWARD(self, t): r'forward'; return t
    def t_UP(self, t): r'up'; return t
    def t_MOVING(self, t): r'moving'; return t
    def t_LEFT(self, t): r'left'; return t
    def t_DOWN(self, t): r'down'; return t
    def t_RIGHT(self, t): r'right'; return t
    def t_LOCATION(self, t): r'location'; return t
    def t_PLACE(self, t): r'place'; return t
    def t_METERS(self, t): r'meters'; return t

    t_ID      = r'[a-zA-Z_][a-zA-Z0-9_]*'

    # A regular expression rule with some action code
    # Note addition of self parameter since we're in a class
    def t_NUMBER(self,t):
        r'\d+'
        t.value = int(t.value)    
        return t

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

m = Lexer()
m.build()           # Build the lexer

if __name__ == '__main__':
    # Build the lexer and try it out
    print("TEST 1")
    m.test("recognize object as id")     # Test it
    print("TEST 2")
    m.test("record enact pose hello as hello_pose")