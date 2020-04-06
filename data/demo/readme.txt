DATA FORMAT

#customers, capacity, distance, (service time)
depot x 	depot y		depot requirement (always 0)
customer x 	customer y	customer requirement
customer x	customer y	customer requirement
.
.
.
customer x	customer y	customer requirement



************************************************************************

DATA SOURCE

Small: http://www.coin-or.org/SYMPHONY/branchandcut/VRP/data/index.htm
Medium and Large: http://www.rhsmith.umd.edu/faculty/bgolden/vrp_data.htm



For 

Small (classical ones): 21 instances including Fisher's.
	Christofides_k_n (k=1,...14, n=21,...199
	Fisher_n (n=44,71, 134).

Medium: 
	Golden_k_240 (k=1,...20) with n=240,...,483

Large ones:
	Li_n: k=21,..32, n=560,...,1200