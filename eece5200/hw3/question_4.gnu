set ylabel "Population / 100,000"
set xlabel "Year / 10"
plot '_actual_vals.temp' using 1:2 title "Actual" w l, \
'_svd_full.temp' using 1:2 title "Full SVD", \
'_svd_reduced.temp' using 1:2 title "Lowest SV Removed"
