\ Model trs0
\ LP format - for model browsing. Use MPS format to capture full model detail.
Minimize
  0 y[Freiburg_im_Breisgau,Freiburg_im_Breisgau,Superman]
   + 0 y[Heidelberg,Heidelberg,Superman] + 0 t[Freiburg_im_Breisgau]
   + 3 z[Customer1] + 3 z[Customer2] + z[Customer3] + 2 z[Customer4]
   + 2 z[Customer5] + 4 z[Customer6] + z[Customer7] + 183 xa[Customer1]
   + 183 xa[Customer2] + 61 xa[Customer3] + 122 xa[Customer4]
   + 122 xa[Customer5] + 244 xa[Customer6] + 61 xa[Customer7]
   + 183 xb[Customer1] + 183 xb[Customer2] + 61 xb[Customer3]
   + 122 xb[Customer4] + 122 xb[Customer5] + 244 xb[Customer6]
   + 61 xb[Customer7] + 18300 g[Customer1] + 18300 g[Customer2]
   + 6100 g[Customer3] + 12200 g[Customer4] + 12200 g[Customer5]
   + 24400 g[Customer6] + 6100 g[Customer7]
Subject To
 assignToJob[Customer1]: x[Customer1,Superman] + g[Customer1] = 1
 assignToJob[Customer2]: x[Customer2,Superman] + g[Customer2] = 1
 assignToJob[Customer3]: g[Customer3] = 1
 assignToJob[Customer4]: g[Customer4] = 1
 assignToJob[Customer5]: g[Customer5] = 1
 assignToJob[Customer6]: g[Customer6] = 1
 assignToJob[Customer7]: x[Customer7,Superman] + g[Customer7] = 1
 assignOne[Customer1]: x[Customer1,Superman] <= 1
 assignOne[Customer2]: x[Customer2,Superman] <= 1
 assignOne[Customer3]: x[Customer3,Superman] <= 1
 assignOne[Customer4]: x[Customer4,Superman] <= 1
 assignOne[Customer5]: x[Customer5,Superman] <= 1
 assignOne[Customer6]: x[Customer6,Superman] <= 1
 assignOne[Customer7]: x[Customer7,Superman] <= 1
 techCapacity[Superman]: 30 x[Customer1,Superman]
   + 30 x[Customer2,Superman] + 60 x[Customer3,Superman]
   + 60 x[Customer4,Superman] + 60 x[Customer5,Superman]
   + 60 x[Customer6,Superman] + 60 x[Customer7,Superman] - 1000 u[Superman]
   + 68 y[Freiburg_im_Breisgau,Baden-Baden,Superman]
   + 85 y[Freiburg_im_Breisgau,Karlsruhe,Superman]
   + 39 y[Freiburg_im_Breisgau,Lahr/Schwarzwald,Superman]
   + 62 y[Freiburg_im_Breisgau,Buehl,Superman]
   + 120 y[Freiburg_im_Breisgau,Heidelberg,Superman]
   + 48 y[Freiburg_im_Breisgau,Loerrach,Superman]
   + 45 y[Freiburg_im_Breisgau,Offenburg,Superman]
   + 125 y[Freiburg_im_Breisgau,Mannheim,Superman]
   + 68 y[Baden-Baden,Freiburg_im_Breisgau,Superman]
   + 31 y[Baden-Baden,Karlsruhe,Superman]
   + 46 y[Baden-Baden,Lahr/Schwarzwald,Superman]
   + 16 y[Baden-Baden,Buehl,Superman]
   + 67 y[Baden-Baden,Heidelberg,Superman]
   + 98 y[Baden-Baden,Loerrach,Superman]
   + 36 y[Baden-Baden,Offenburg,Superman]
   + 74 y[Baden-Baden,Mannheim,Superman]
   + 85 y[Karlsruhe,Freiburg_im_Breisgau,Superman]
   + 31 y[Karlsruhe,Baden-Baden,Superman]
   + 61 y[Karlsruhe,Lahr/Schwarzwald,Superman]
   + 35 y[Karlsruhe,Buehl,Superman] + 50 y[Karlsruhe,Heidelberg,Superman]
   + 115 y[Karlsruhe,Loerrach,Superman]
   + 51 y[Karlsruhe,Offenburg,Superman] + 53 y[Karlsruhe,Mannheim,Superman]
   + 39 y[Lahr/Schwarzwald,Freiburg_im_Breisgau,Superman]
   + 46 y[Lahr/Schwarzwald,Baden-Baden,Superman]
   + 61 y[Lahr/Schwarzwald,Karlsruhe,Superman]
   + 40 y[Lahr/Schwarzwald,Buehl,Superman]
   + 98 y[Lahr/Schwarzwald,Heidelberg,Superman]
   + 70 y[Lahr/Schwarzwald,Loerrach,Superman]
   + 26 y[Lahr/Schwarzwald,Offenburg,Superman]
   + 106 y[Lahr/Schwarzwald,Mannheim,Superman]
   + 62 y[Buehl,Freiburg_im_Breisgau,Superman]
   + 16 y[Buehl,Baden-Baden,Superman] + 35 y[Buehl,Karlsruhe,Superman]
   + 40 y[Buehl,Lahr/Schwarzwald,Superman]
   + 71 y[Buehl,Heidelberg,Superman] + 92 y[Buehl,Loerrach,Superman]
   + 30 y[Buehl,Offenburg,Superman] + 77 y[Buehl,Mannheim,Superman]
   + 120 y[Heidelberg,Freiburg_im_Breisgau,Superman]
   + 67 y[Heidelberg,Baden-Baden,Superman]
   + 50 y[Heidelberg,Karlsruhe,Superman]
   + 98 y[Heidelberg,Lahr/Schwarzwald,Superman]
   + 71 y[Heidelberg,Buehl,Superman] + 150 y[Heidelberg,Loerrach,Superman]
   + 88 y[Heidelberg,Offenburg,Superman]
   + 24 y[Heidelberg,Mannheim,Superman]
   + 48 y[Loerrach,Freiburg_im_Breisgau,Superman]
   + 98 y[Loerrach,Baden-Baden,Superman]
   + 115 y[Loerrach,Karlsruhe,Superman]
   + 70 y[Loerrach,Lahr/Schwarzwald,Superman]
   + 92 y[Loerrach,Buehl,Superman] + 150 y[Loerrach,Heidelberg,Superman]
   + 80 y[Loerrach,Offenburg,Superman] + 160 y[Loerrach,Mannheim,Superman]
   + 45 y[Offenburg,Freiburg_im_Breisgau,Superman]
   + 36 y[Offenburg,Baden-Baden,Superman]
   + 51 y[Offenburg,Karlsruhe,Superman]
   + 26 y[Offenburg,Lahr/Schwarzwald,Superman]
   + 30 y[Offenburg,Buehl,Superman] + 88 y[Offenburg,Heidelberg,Superman]
   + 80 y[Offenburg,Loerrach,Superman] + 95 y[Offenburg,Mannheim,Superman]
   + 125 y[Mannheim,Freiburg_im_Breisgau,Superman]
   + 74 y[Mannheim,Baden-Baden,Superman]
   + 53 y[Mannheim,Karlsruhe,Superman]
   + 106 y[Mannheim,Lahr/Schwarzwald,Superman]
   + 77 y[Mannheim,Buehl,Superman] + 24 y[Mannheim,Heidelberg,Superman]
   + 160 y[Mannheim,Loerrach,Superman] + 95 y[Mannheim,Offenburg,Superman]
   <= 0
 techTour1[Superman,Customer1]: - x[Customer1,Superman]
   + y[Freiburg_im_Breisgau,Mannheim,Superman]
   + y[Baden-Baden,Mannheim,Superman] + y[Karlsruhe,Mannheim,Superman]
   + y[Lahr/Schwarzwald,Mannheim,Superman] + y[Buehl,Mannheim,Superman]
   + y[Heidelberg,Mannheim,Superman] + y[Loerrach,Mannheim,Superman]
   + y[Offenburg,Mannheim,Superman] + y[Mannheim,Mannheim,Superman] = 0
 techTour1[Superman,Customer2]: - x[Customer2,Superman]
   + y[Freiburg_im_Breisgau,Karlsruhe,Superman]
   + y[Baden-Baden,Karlsruhe,Superman] + y[Karlsruhe,Karlsruhe,Superman]
   + y[Lahr/Schwarzwald,Karlsruhe,Superman] + y[Buehl,Karlsruhe,Superman]
   + y[Heidelberg,Karlsruhe,Superman] + y[Loerrach,Karlsruhe,Superman]
   + y[Offenburg,Karlsruhe,Superman] + y[Mannheim,Karlsruhe,Superman] = 0
 techTour1[Superman,Customer3]: - x[Customer3,Superman]
   + y[Freiburg_im_Breisgau,Baden-Baden,Superman]
   + y[Baden-Baden,Baden-Baden,Superman]
   + y[Karlsruhe,Baden-Baden,Superman]
   + y[Lahr/Schwarzwald,Baden-Baden,Superman]
   + y[Buehl,Baden-Baden,Superman] + y[Heidelberg,Baden-Baden,Superman]
   + y[Loerrach,Baden-Baden,Superman] + y[Offenburg,Baden-Baden,Superman]
   + y[Mannheim,Baden-Baden,Superman] = 0
 techTour1[Superman,Customer4]: - x[Customer4,Superman]
   + y[Freiburg_im_Breisgau,Buehl,Superman] + y[Baden-Baden,Buehl,Superman]
   + y[Karlsruhe,Buehl,Superman] + y[Lahr/Schwarzwald,Buehl,Superman]
   + y[Buehl,Buehl,Superman] + y[Heidelberg,Buehl,Superman]
   + y[Loerrach,Buehl,Superman] + y[Offenburg,Buehl,Superman]
   + y[Mannheim,Buehl,Superman] = 0
 techTour1[Superman,Customer5]: - x[Customer5,Superman]
   + y[Freiburg_im_Breisgau,Offenburg,Superman]
   + y[Baden-Baden,Offenburg,Superman] + y[Karlsruhe,Offenburg,Superman]
   + y[Lahr/Schwarzwald,Offenburg,Superman] + y[Buehl,Offenburg,Superman]
   + y[Heidelberg,Offenburg,Superman] + y[Loerrach,Offenburg,Superman]
   + y[Offenburg,Offenburg,Superman] + y[Mannheim,Offenburg,Superman] = 0
 techTour1[Superman,Customer6]: - x[Customer6,Superman]
   + y[Freiburg_im_Breisgau,Lahr/Schwarzwald,Superman]
   + y[Baden-Baden,Lahr/Schwarzwald,Superman]
   + y[Karlsruhe,Lahr/Schwarzwald,Superman]
   + y[Lahr/Schwarzwald,Lahr/Schwarzwald,Superman]
   + y[Buehl,Lahr/Schwarzwald,Superman]
   + y[Heidelberg,Lahr/Schwarzwald,Superman]
   + y[Loerrach,Lahr/Schwarzwald,Superman]
   + y[Offenburg,Lahr/Schwarzwald,Superman]
   + y[Mannheim,Lahr/Schwarzwald,Superman] = 0
 techTour1[Superman,Customer7]: - x[Customer7,Superman]
   + y[Freiburg_im_Breisgau,Loerrach,Superman]
   + y[Baden-Baden,Loerrach,Superman] + y[Karlsruhe,Loerrach,Superman]
   + y[Lahr/Schwarzwald,Loerrach,Superman] + y[Buehl,Loerrach,Superman]
   + y[Heidelberg,Loerrach,Superman] + y[Loerrach,Loerrach,Superman]
   + y[Offenburg,Loerrach,Superman] + y[Mannheim,Loerrach,Superman] = 0
 techTour2[Superman,Customer1]: - x[Customer1,Superman]
   + y[Mannheim,Freiburg_im_Breisgau,Superman]
   + y[Mannheim,Baden-Baden,Superman] + y[Mannheim,Karlsruhe,Superman]
   + y[Mannheim,Lahr/Schwarzwald,Superman] + y[Mannheim,Buehl,Superman]
   + y[Mannheim,Heidelberg,Superman] + y[Mannheim,Loerrach,Superman]
   + y[Mannheim,Offenburg,Superman] + y[Mannheim,Mannheim,Superman] = 0
 techTour2[Superman,Customer2]: - x[Customer2,Superman]
   + y[Karlsruhe,Freiburg_im_Breisgau,Superman]
   + y[Karlsruhe,Baden-Baden,Superman] + y[Karlsruhe,Karlsruhe,Superman]
   + y[Karlsruhe,Lahr/Schwarzwald,Superman] + y[Karlsruhe,Buehl,Superman]
   + y[Karlsruhe,Heidelberg,Superman] + y[Karlsruhe,Loerrach,Superman]
   + y[Karlsruhe,Offenburg,Superman] + y[Karlsruhe,Mannheim,Superman] = 0
 techTour2[Superman,Customer3]: - x[Customer3,Superman]
   + y[Baden-Baden,Freiburg_im_Breisgau,Superman]
   + y[Baden-Baden,Baden-Baden,Superman]
   + y[Baden-Baden,Karlsruhe,Superman]
   + y[Baden-Baden,Lahr/Schwarzwald,Superman]
   + y[Baden-Baden,Buehl,Superman] + y[Baden-Baden,Heidelberg,Superman]
   + y[Baden-Baden,Loerrach,Superman] + y[Baden-Baden,Offenburg,Superman]
   + y[Baden-Baden,Mannheim,Superman] = 0
 techTour2[Superman,Customer4]: - x[Customer4,Superman]
   + y[Buehl,Freiburg_im_Breisgau,Superman] + y[Buehl,Baden-Baden,Superman]
   + y[Buehl,Karlsruhe,Superman] + y[Buehl,Lahr/Schwarzwald,Superman]
   + y[Buehl,Buehl,Superman] + y[Buehl,Heidelberg,Superman]
   + y[Buehl,Loerrach,Superman] + y[Buehl,Offenburg,Superman]
   + y[Buehl,Mannheim,Superman] = 0
 techTour2[Superman,Customer5]: - x[Customer5,Superman]
   + y[Offenburg,Freiburg_im_Breisgau,Superman]
   + y[Offenburg,Baden-Baden,Superman] + y[Offenburg,Karlsruhe,Superman]
   + y[Offenburg,Lahr/Schwarzwald,Superman] + y[Offenburg,Buehl,Superman]
   + y[Offenburg,Heidelberg,Superman] + y[Offenburg,Loerrach,Superman]
   + y[Offenburg,Offenburg,Superman] + y[Offenburg,Mannheim,Superman] = 0
 techTour2[Superman,Customer6]: - x[Customer6,Superman]
   + y[Lahr/Schwarzwald,Freiburg_im_Breisgau,Superman]
   + y[Lahr/Schwarzwald,Baden-Baden,Superman]
   + y[Lahr/Schwarzwald,Karlsruhe,Superman]
   + y[Lahr/Schwarzwald,Lahr/Schwarzwald,Superman]
   + y[Lahr/Schwarzwald,Buehl,Superman]
   + y[Lahr/Schwarzwald,Heidelberg,Superman]
   + y[Lahr/Schwarzwald,Loerrach,Superman]
   + y[Lahr/Schwarzwald,Offenburg,Superman]
   + y[Lahr/Schwarzwald,Mannheim,Superman] = 0
 techTour2[Superman,Customer7]: - x[Customer7,Superman]
   + y[Loerrach,Freiburg_im_Breisgau,Superman]
   + y[Loerrach,Baden-Baden,Superman] + y[Loerrach,Karlsruhe,Superman]
   + y[Loerrach,Lahr/Schwarzwald,Superman] + y[Loerrach,Buehl,Superman]
   + y[Loerrach,Heidelberg,Superman] + y[Loerrach,Loerrach,Superman]
   + y[Loerrach,Offenburg,Superman] + y[Loerrach,Mannheim,Superman] = 0
 sameDepot1[Superman]: - u[Superman] + y[Baden-Baden,Heidelberg,Superman]
   + y[Karlsruhe,Heidelberg,Superman]
   + y[Lahr/Schwarzwald,Heidelberg,Superman] + y[Buehl,Heidelberg,Superman]
   + y[Loerrach,Heidelberg,Superman] + y[Offenburg,Heidelberg,Superman]
   + y[Mannheim,Heidelberg,Superman] = 0
 sameDepot2[Superman]: - u[Superman] + y[Heidelberg,Baden-Baden,Superman]
   + y[Heidelberg,Karlsruhe,Superman]
   + y[Heidelberg,Lahr/Schwarzwald,Superman] + y[Heidelberg,Buehl,Superman]
   + y[Heidelberg,Loerrach,Superman] + y[Heidelberg,Offenburg,Superman]
   + y[Heidelberg,Mannheim,Superman] = 0
 tempoCustomer[Customer1,Customer1]: - 630 y[Mannheim,Mannheim,Superman]
   >= -600
 tempoCustomer[Customer1,Customer2]: - 683 y[Mannheim,Karlsruhe,Superman]
   + t[Karlsruhe] - t[Mannheim] >= -600
 tempoCustomer[Customer1,Customer3]: - 704 y[Mannheim,Baden-Baden,Superman]
   + t[Baden-Baden] - t[Mannheim] >= -600
 tempoCustomer[Customer1,Customer4]: - 707 y[Mannheim,Buehl,Superman]
   + t[Buehl] - t[Mannheim] >= -600
 tempoCustomer[Customer1,Customer5]: - 725 y[Mannheim,Offenburg,Superman]
   + t[Offenburg] - t[Mannheim] >= -600
 tempoCustomer[Customer1,Customer6]:
   - 736 y[Mannheim,Lahr/Schwarzwald,Superman] + t[Lahr/Schwarzwald]
   - t[Mannheim] >= -600
 tempoCustomer[Customer1,Customer7]: - 790 y[Mannheim,Loerrach,Superman]
   + t[Loerrach] - t[Mannheim] >= -600
 tempoCustomer[Customer2,Customer1]: - 683 y[Karlsruhe,Mannheim,Superman]
   - t[Karlsruhe] + t[Mannheim] >= -600
 tempoCustomer[Customer2,Customer2]: - 630 y[Karlsruhe,Karlsruhe,Superman]
   >= -600
 tempoCustomer[Customer2,Customer3]:
   - 661 y[Karlsruhe,Baden-Baden,Superman] + t[Baden-Baden] - t[Karlsruhe]
   >= -600
 tempoCustomer[Customer2,Customer4]: - 665 y[Karlsruhe,Buehl,Superman]
   - t[Karlsruhe] + t[Buehl] >= -600
 tempoCustomer[Customer2,Customer5]: - 681 y[Karlsruhe,Offenburg,Superman]
   - t[Karlsruhe] + t[Offenburg] >= -600
 tempoCustomer[Customer2,Customer6]:
   - 691 y[Karlsruhe,Lahr/Schwarzwald,Superman] - t[Karlsruhe]
   + t[Lahr/Schwarzwald] >= -600
 tempoCustomer[Customer2,Customer7]: - 745 y[Karlsruhe,Loerrach,Superman]
   - t[Karlsruhe] + t[Loerrach] >= -600
 tempoCustomer[Customer3,Customer1]: - 734 y[Baden-Baden,Mannheim,Superman]
   - t[Baden-Baden] + t[Mannheim] >= -600
 tempoCustomer[Customer3,Customer2]:
   - 691 y[Baden-Baden,Karlsruhe,Superman] - t[Baden-Baden] + t[Karlsruhe]
   >= -600
 tempoCustomer[Customer3,Customer3]:
   - 660 y[Baden-Baden,Baden-Baden,Superman] >= -600
 tempoCustomer[Customer3,Customer4]: - 676 y[Baden-Baden,Buehl,Superman]
   - t[Baden-Baden] + t[Buehl] >= -600
 tempoCustomer[Customer3,Customer5]:
   - 696 y[Baden-Baden,Offenburg,Superman] - t[Baden-Baden] + t[Offenburg]
   >= -600
 tempoCustomer[Customer3,Customer6]:
   - 706 y[Baden-Baden,Lahr/Schwarzwald,Superman] - t[Baden-Baden]
   + t[Lahr/Schwarzwald] >= -600
 tempoCustomer[Customer3,Customer7]: - 758 y[Baden-Baden,Loerrach,Superman]
   - t[Baden-Baden] + t[Loerrach] >= -600
 tempoCustomer[Customer4,Customer1]: - 737 y[Buehl,Mannheim,Superman]
   - t[Buehl] + t[Mannheim] >= -600
 tempoCustomer[Customer4,Customer2]: - 695 y[Buehl,Karlsruhe,Superman]
   + t[Karlsruhe] - t[Buehl] >= -600
 tempoCustomer[Customer4,Customer3]: - 676 y[Buehl,Baden-Baden,Superman]
   + t[Baden-Baden] - t[Buehl] >= -600
 tempoCustomer[Customer4,Customer4]: - 660 y[Buehl,Buehl,Superman] >= -600
 tempoCustomer[Customer4,Customer5]: - 690 y[Buehl,Offenburg,Superman]
   - t[Buehl] + t[Offenburg] >= -600
 tempoCustomer[Customer4,Customer6]:
   - 700 y[Buehl,Lahr/Schwarzwald,Superman] + t[Lahr/Schwarzwald]
   - t[Buehl] >= -600
 tempoCustomer[Customer4,Customer7]: - 752 y[Buehl,Loerrach,Superman]
   - t[Buehl] + t[Loerrach] >= -600
 tempoCustomer[Customer5,Customer1]: - 755 y[Offenburg,Mannheim,Superman]
   - t[Offenburg] + t[Mannheim] >= -600
 tempoCustomer[Customer5,Customer2]: - 711 y[Offenburg,Karlsruhe,Superman]
   + t[Karlsruhe] - t[Offenburg] >= -600
 tempoCustomer[Customer5,Customer3]:
   - 696 y[Offenburg,Baden-Baden,Superman] + t[Baden-Baden] - t[Offenburg]
   >= -600
 tempoCustomer[Customer5,Customer4]: - 690 y[Offenburg,Buehl,Superman]
   + t[Buehl] - t[Offenburg] >= -600
 tempoCustomer[Customer5,Customer5]: - 660 y[Offenburg,Offenburg,Superman]
   >= -600
 tempoCustomer[Customer5,Customer6]:
   - 686 y[Offenburg,Lahr/Schwarzwald,Superman] + t[Lahr/Schwarzwald]
   - t[Offenburg] >= -600
 tempoCustomer[Customer5,Customer7]: - 740 y[Offenburg,Loerrach,Superman]
   + t[Loerrach] - t[Offenburg] >= -600
 tempoCustomer[Customer6,Customer1]:
   - 766 y[Lahr/Schwarzwald,Mannheim,Superman] - t[Lahr/Schwarzwald]
   + t[Mannheim] >= -600
 tempoCustomer[Customer6,Customer2]:
   - 721 y[Lahr/Schwarzwald,Karlsruhe,Superman] + t[Karlsruhe]
   - t[Lahr/Schwarzwald] >= -600
 tempoCustomer[Customer6,Customer3]:
   - 706 y[Lahr/Schwarzwald,Baden-Baden,Superman] + t[Baden-Baden]
   - t[Lahr/Schwarzwald] >= -600
 tempoCustomer[Customer6,Customer4]:
   - 700 y[Lahr/Schwarzwald,Buehl,Superman] - t[Lahr/Schwarzwald]
   + t[Buehl] >= -600
 tempoCustomer[Customer6,Customer5]:
   - 686 y[Lahr/Schwarzwald,Offenburg,Superman] - t[Lahr/Schwarzwald]
   + t[Offenburg] >= -600
 tempoCustomer[Customer6,Customer6]:
   - 660 y[Lahr/Schwarzwald,Lahr/Schwarzwald,Superman] >= -600
 tempoCustomer[Customer6,Customer7]:
   - 730 y[Lahr/Schwarzwald,Loerrach,Superman] - t[Lahr/Schwarzwald]
   + t[Loerrach] >= -600
 tempoCustomer[Customer7,Customer1]: - 820 y[Loerrach,Mannheim,Superman]
   - t[Loerrach] + t[Mannheim] >= -600
 tempoCustomer[Customer7,Customer2]: - 775 y[Loerrach,Karlsruhe,Superman]
   + t[Karlsruhe] - t[Loerrach] >= -600
 tempoCustomer[Customer7,Customer3]: - 758 y[Loerrach,Baden-Baden,Superman]
   + t[Baden-Baden] - t[Loerrach] >= -600
 tempoCustomer[Customer7,Customer4]: - 752 y[Loerrach,Buehl,Superman]
   + t[Buehl] - t[Loerrach] >= -600
 tempoCustomer[Customer7,Customer5]: - 740 y[Loerrach,Offenburg,Superman]
   - t[Loerrach] + t[Offenburg] >= -600
 tempoCustomer[Customer7,Customer6]:
   - 730 y[Loerrach,Lahr/Schwarzwald,Superman] + t[Lahr/Schwarzwald]
   - t[Loerrach] >= -600
 tempoCustomer[Customer7,Customer7]: - 660 y[Loerrach,Loerrach,Superman]
   >= -600
 tempoDepot[Heidelberg,Customer1]: - 624 y[Heidelberg,Mannheim,Superman]
   - t[Heidelberg] + t[Mannheim] >= -600
 tempoDepot[Heidelberg,Customer2]: - 650 y[Heidelberg,Karlsruhe,Superman]
   + t[Karlsruhe] - t[Heidelberg] >= -600
 tempoDepot[Heidelberg,Customer3]: - 667 y[Heidelberg,Baden-Baden,Superman]
   + t[Baden-Baden] - t[Heidelberg] >= -600
 tempoDepot[Heidelberg,Customer4]: - 671 y[Heidelberg,Buehl,Superman]
   + t[Buehl] - t[Heidelberg] >= -600
 tempoDepot[Heidelberg,Customer5]: - 688 y[Heidelberg,Offenburg,Superman]
   - t[Heidelberg] + t[Offenburg] >= -600
 tempoDepot[Heidelberg,Customer6]:
   - 698 y[Heidelberg,Lahr/Schwarzwald,Superman] + t[Lahr/Schwarzwald]
   - t[Heidelberg] >= -600
 tempoDepot[Heidelberg,Customer7]: - 750 y[Heidelberg,Loerrach,Superman]
   - t[Heidelberg] + t[Loerrach] >= -600
 timeWinA[Customer1]: t[Mannheim] + xa[Customer1] >= 0
 timeWinA[Customer2]: t[Karlsruhe] + xa[Customer2] >= 30
 timeWinA[Customer3]: t[Baden-Baden] + xa[Customer3] >= 60
 timeWinA[Customer4]: t[Buehl] + xa[Customer4] >= 120
 timeWinA[Customer5]: t[Offenburg] + xa[Customer5] >= 240
 timeWinA[Customer6]: t[Lahr/Schwarzwald] + xa[Customer6] >= 300
 timeWinA[Customer7]: t[Loerrach] + xa[Customer7] >= 360
 timeWinB[Customer1]: t[Mannheim] - xb[Customer1] <= 30
 timeWinB[Customer2]: t[Karlsruhe] - xb[Customer2] <= 150
 timeWinB[Customer3]: t[Baden-Baden] - xb[Customer3] <= 180
 timeWinB[Customer4]: t[Buehl] - xb[Customer4] <= 240
 timeWinB[Customer5]: t[Offenburg] - xb[Customer5] <= 360
 timeWinB[Customer6]: t[Lahr/Schwarzwald] - xb[Customer6] <= 420
 timeWinB[Customer7]: t[Loerrach] - xb[Customer7] <= 480
 lateness[Customer1]: - t[Mannheim] + z[Customer1] >= -30
 lateness[Customer2]: - t[Karlsruhe] + z[Customer2] >= -150
 lateness[Customer3]: - t[Baden-Baden] + z[Customer3] >= -180
 lateness[Customer4]: - t[Buehl] + z[Customer4] >= -240
 lateness[Customer5]: - t[Offenburg] + z[Customer5] >= -360
 lateness[Customer6]: - t[Lahr/Schwarzwald] + z[Customer6] >= -420
 lateness[Customer7]: - t[Loerrach] + z[Customer7] >= -480
Bounds
 t[Freiburg_im_Breisgau] <= 600
 t[Baden-Baden] <= 600
 t[Karlsruhe] <= 600
 t[Lahr/Schwarzwald] <= 600
 t[Buehl] <= 600
 t[Heidelberg] <= 600
 t[Loerrach] <= 600
 t[Offenburg] <= 600
 t[Mannheim] <= 600
Binaries
 x[Customer1,Superman] x[Customer2,Superman] x[Customer3,Superman]
 x[Customer4,Superman] x[Customer5,Superman] x[Customer6,Superman]
 x[Customer7,Superman] u[Superman]
 y[Freiburg_im_Breisgau,Freiburg_im_Breisgau,Superman]
 y[Freiburg_im_Breisgau,Baden-Baden,Superman]
 y[Freiburg_im_Breisgau,Karlsruhe,Superman]
 y[Freiburg_im_Breisgau,Lahr/Schwarzwald,Superman]
 y[Freiburg_im_Breisgau,Buehl,Superman]
 y[Freiburg_im_Breisgau,Heidelberg,Superman]
 y[Freiburg_im_Breisgau,Loerrach,Superman]
 y[Freiburg_im_Breisgau,Offenburg,Superman]
 y[Freiburg_im_Breisgau,Mannheim,Superman]
 y[Baden-Baden,Freiburg_im_Breisgau,Superman]
 y[Baden-Baden,Baden-Baden,Superman] y[Baden-Baden,Karlsruhe,Superman]
 y[Baden-Baden,Lahr/Schwarzwald,Superman] y[Baden-Baden,Buehl,Superman]
 y[Baden-Baden,Heidelberg,Superman] y[Baden-Baden,Loerrach,Superman]
 y[Baden-Baden,Offenburg,Superman] y[Baden-Baden,Mannheim,Superman]
 y[Karlsruhe,Freiburg_im_Breisgau,Superman]
 y[Karlsruhe,Baden-Baden,Superman] y[Karlsruhe,Karlsruhe,Superman]
 y[Karlsruhe,Lahr/Schwarzwald,Superman] y[Karlsruhe,Buehl,Superman]
 y[Karlsruhe,Heidelberg,Superman] y[Karlsruhe,Loerrach,Superman]
 y[Karlsruhe,Offenburg,Superman] y[Karlsruhe,Mannheim,Superman]
 y[Lahr/Schwarzwald,Freiburg_im_Breisgau,Superman]
 y[Lahr/Schwarzwald,Baden-Baden,Superman]
 y[Lahr/Schwarzwald,Karlsruhe,Superman]
 y[Lahr/Schwarzwald,Lahr/Schwarzwald,Superman]
 y[Lahr/Schwarzwald,Buehl,Superman] y[Lahr/Schwarzwald,Heidelberg,Superman]
 y[Lahr/Schwarzwald,Loerrach,Superman]
 y[Lahr/Schwarzwald,Offenburg,Superman]
 y[Lahr/Schwarzwald,Mannheim,Superman]
 y[Buehl,Freiburg_im_Breisgau,Superman] y[Buehl,Baden-Baden,Superman]
 y[Buehl,Karlsruhe,Superman] y[Buehl,Lahr/Schwarzwald,Superman]
 y[Buehl,Buehl,Superman] y[Buehl,Heidelberg,Superman]
 y[Buehl,Loerrach,Superman] y[Buehl,Offenburg,Superman]
 y[Buehl,Mannheim,Superman] y[Heidelberg,Freiburg_im_Breisgau,Superman]
 y[Heidelberg,Baden-Baden,Superman] y[Heidelberg,Karlsruhe,Superman]
 y[Heidelberg,Lahr/Schwarzwald,Superman] y[Heidelberg,Buehl,Superman]
 y[Heidelberg,Heidelberg,Superman] y[Heidelberg,Loerrach,Superman]
 y[Heidelberg,Offenburg,Superman] y[Heidelberg,Mannheim,Superman]
 y[Loerrach,Freiburg_im_Breisgau,Superman] y[Loerrach,Baden-Baden,Superman]
 y[Loerrach,Karlsruhe,Superman] y[Loerrach,Lahr/Schwarzwald,Superman]
 y[Loerrach,Buehl,Superman] y[Loerrach,Heidelberg,Superman]
 y[Loerrach,Loerrach,Superman] y[Loerrach,Offenburg,Superman]
 y[Loerrach,Mannheim,Superman] y[Offenburg,Freiburg_im_Breisgau,Superman]
 y[Offenburg,Baden-Baden,Superman] y[Offenburg,Karlsruhe,Superman]
 y[Offenburg,Lahr/Schwarzwald,Superman] y[Offenburg,Buehl,Superman]
 y[Offenburg,Heidelberg,Superman] y[Offenburg,Loerrach,Superman]
 y[Offenburg,Offenburg,Superman] y[Offenburg,Mannheim,Superman]
 y[Mannheim,Freiburg_im_Breisgau,Superman] y[Mannheim,Baden-Baden,Superman]
 y[Mannheim,Karlsruhe,Superman] y[Mannheim,Lahr/Schwarzwald,Superman]
 y[Mannheim,Buehl,Superman] y[Mannheim,Heidelberg,Superman]
 y[Mannheim,Loerrach,Superman] y[Mannheim,Offenburg,Superman]
 y[Mannheim,Mannheim,Superman] g[Customer1] g[Customer2] g[Customer3]
 g[Customer4] g[Customer5] g[Customer6] g[Customer7]
End
