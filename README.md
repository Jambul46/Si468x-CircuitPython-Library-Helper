CircuitPython Library/Driver/Helper… whatever… designed to simplify using Silabs/Skyworks Si468x FM/HD/DAB receivers.

Currently, there’s no DAB support, as coverage is poor where I live, so I can’t test it.  
However, you *can* use `send_command(cmd, value in bytearray)` to access the IC directly—basically, the whole chip is supported; the library just doesn’t provide dedicated shortcut functions yet.

For anything related to the software side of the Si468x, refer to AN649: Si468x Programming Guide.

About this "thing":  
I’m not a professional coder, so if an actual programmer stumbles across this, I apologize in advance.  
No, this was not written by ChatGPT—I did ask for a little help, but that’s it.  
I seem to have a knack for connecting examples and snippets I find online, and that’s how this "thing" came to life.
