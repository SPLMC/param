# PARAMetric Markov Model Analyser

Fork of the excelent PARAMetric Markov Model Analyzer (PARAM Tool v2.3).

This fork works around the eventual "Error compiling Lua script: too many
syntax levels". This error happened in models with modules larger than 200
commands, and was due to the synthesis of a command which was basically a
conjunction of the negations of each command's guard (a _sink_ guard).

The adopted solution was to enable/disable this behavior with a command-line switch
(`--fix-deadlocks`, honoring the name of the method which implements this).
The default behavior is now to not synthesize this sink command. If you happen
to be using this fork and this strategy does not work for you, turn on the switch
and give it a try.

Credits
-------

PARAM was originally developed by Ernst Moritz Hahn, Holger Hermanns, Lijun Zhang and Björn Wachter.
