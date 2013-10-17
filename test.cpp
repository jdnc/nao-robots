// construct only horizontal runs
//construct only for given color
void connectComponents(Color rkcolor){
    // do it for each row
    //find the parent
    VisionPoint **rle_map = classifier_.horizontalPoint[rkcolor];
    uint32_t *count_map = classifier_.horizontalPointCount[rkcolor];
    uint32_t lim1, lim2, l1, l2;
    l1 = l2 = 0;
    VisionPoint *n, *p;
    //find the max runs for this color
    int max_run = 0;
    for(int i=0; i<iparams_.height; i++) max_run += count_map[i];
    VisionPoint *s = &rle_map[1][0];
    VisionPoint *r2 = &rle_map[0][0];
    VisionPoint *r1 = &rle_map[1][0];
    
    for (int i=0; i<iparams_.height; i++){
    lim2 = count_map[i];
    lim1 = count_map[i+1];
    while(l1 < lim1 && l2 < lim2){
    /*if (lim1 == 0 or lim2 == 0){
    //consider next two adjacent rows
    i++;
    continue;
    }*/
    VisionPoint *r2 = &rle_map[i][l2];
    VisionPoint *r1 = &rle_map[i+1][l1];
    if(r1 && r2 && rkcolor){
    //they are not zero - so just check four connectedness
    if((r1->xi >= r2->xi && r1->xi < r2->xf) || (r2->xi >= r1->xi && r2->xi < r1->xf)) {
      if(s != r1){
        s->parent = r1->parent = r2->parent;
        s = r2;
      }
      else{
        n = r1->parent;
        while(n->parent != n) n = n->parent;
        p = r2->parent;
        while(p->parent != p) p = p->parent;
        //find the smaller one 
        if (n->yi < p->yi){
          p->parent = n;
        }
        else if(n->xi < p->xi){
          p->parent = n;
        }
        else{
          n->parent = p;
        }
      }
    }
    }
    else if (r1 && r2){
      if (r1->xf < r2->xf) l1++;
      else l2++;
    }
      
    }
    }
    for(int i=0; i<iparams_.height; i++){
      for(int lim=0; lim<count_map[i]; lim++){
        n = &rle_map[i][lim];
        p = n->parent;
        if (p->yi > n->yi){
          while(p != p->parent) p = p->parent;
          n->parent = p;
        }
      }
    }
}
