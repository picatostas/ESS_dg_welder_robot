###
#  Python
#  Implementation of Jose M Moya fsm C library
#  Originally published under GPL v3
#
#
#  Author : Pablo Costas Franco
# 
#
###


class fsm:
    def __init__(self, trans_table, init_state):
    	self.current_state = init_state
        self.trans_table   = trans_table

    def fire(self):
        for trans in self.trans_table:
            if(self.current_state == trans.orig_state) and (trans.cond()):
                self.current_state = trans.dest_state
                if trans.trans_callback is not None:    
                    trans.trans_callback() 
                break


class fsm_trans:
    def __init__(self, orig, cond, dest, callback): 
    	self.orig_state = orig
    	self.cond = cond
    	self.dest_state = dest
    	self.trans_callback = callback

