# This message contains information on a error raised after a user update of some CLE related source code:
# Transfer function, Smach script, PyNN script
int32 severity # error severity according to the following constants
int32 SEVERITY_WARNING=0  # The error might not affect the simulation at all
int32 SEVERITY_ERROR=1    # The error will lead to a simulation failure if not resolved
int32 SEVERITY_CRITICAL=2 # The error has lead to simulation failure

string sourceType # e.g., "Transfer Function"
string errorType # e.g., "NoOrMultipleNames", "Compile", "Loading", "Runtime"
string message # description of the error, e.g., "IndentationError: unexpected indent"

string functionName # python def name of the function causing the error, empty if unavailable

# the following fields are used when a python SyntaxError is raised, strings are left empty otherwise
int32 lineNumber # line number of the error, -1 if unavailable 
int32 offset # python's SyntaxError offset, -1 if unavailable
string lineText # text of the line causing the error
string fileName # name of the file in which the error is raised (<string> if no such file exists)
