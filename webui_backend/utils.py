# Copyright 2024 Sonardyne

# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
# documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
# Software.

# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
# OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import sys
import constants
 
def bitshift16(first, second):
    return first | second <<  constants.BITSHIFT_8

def bitshift32(first, second, third, fourth):
    return first | second <<  constants.BITSHIFT_8 | third << constants.BITSHIFT_16 | fourth << (constants.BITSHIFT_24)

def bitshift64(first, second, third, fourth, fifth, sixth, seventh, eighth):
    return first | second <<  constants.BITSHIFT_8 | third << constants.BITSHIFT_16 | fourth << constants.BITSHIFT_24 | fifth << constants.BITSHIFT_32 | sixth << constants.BITSHIFT_40 | seventh << constants.BITSHIFT_48 | eighth << constants.BITSHIFT_56

def twosComplement(value, numberBytes):
    b = value.to_bytes(numberBytes, byteorder=sys.byteorder, signed=False)                                                          
    return int.from_bytes(b, byteorder=sys.byteorder, signed=True)