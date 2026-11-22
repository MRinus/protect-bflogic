# t1b_log_en.sed - Cue-044: maps the German startup-log texts of the
# reference module (ref/bflogic_protect.deployed.c) to the English texts
# of src/. T1b applies it to the reference log only; everything else in
# the log must still be byte-identical.
s/ (partielle Chain)/ (partial chain)/
s/ (frequenzgewichtet)/ (frequency-weighted)/
s/ -- waere wirkungslos\./ -- it would have no effect./
s/: skaliert auf Puffereinheiten: \([^ ]*\) (output_scale=\([^,]*\), entspricht weiterhin /: scaled to buffer units: \1 (output_scale=\2, still /
