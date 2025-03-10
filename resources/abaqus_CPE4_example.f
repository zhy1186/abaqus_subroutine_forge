c***********************************************************
      subroutine uelmat(rhs,amatrx,svars,energy,ndofel,nrhs,
     1     nsvars,props,nprops,coords,mcrd,nnode,u,du, 
     2     v,a,jtype,time,dtime,kstep,kinc,jelem,params, 
     3     ndload,jdltyp,adlmag,predef,npredf,lflags,mlvarx, 
     4     ddlmag,mdload,pnewdt,jprops,njpro,period,
     5     materiallib)
c
      include 'aba_param.inc'
C
      dimension rhs(mlvarx,*), amatrx(ndofel, ndofel), props(*),
     1  svars(*), energy(*), coords(mcrd, nnode), u(ndofel),
     2  du(mlvarx,*), v(ndofel), a(ndofel), time(2), params(*),
     3  jdltyp(mdload,*), adlmag(mdload,*), ddlmag(mdload,*),
     4  predef(2, npredf, nnode), lflags(*), jprops(*)
      parameter (zero=0.d0, dmone=-1.0d0, one=1.d0, four=4.0d0, 
     1     fourth=0.25d0,gaussCoord=0.577350269d0)
      parameter (ndim=2, ndof=2, nshr=1,nnodemax=4,
     1     ntens=4, ninpt=4, nsvint=4)
c
c      ndim  ... number of spatial dimensions
c      ndof  ... number of degrees of freedom per node
c      nshr  ... number of shear stress component
c      ntens ... total number of stress tensor components
c                (=ndi+nshr)
c      ninpt ... number of integration points
c      nsvint... number of state variables per integration pt 
c                (strain)
c
      dimension  stiff(ndof*nnodemax,ndof*nnodemax),
     1 force(ndof*nnodemax), shape(nnodemax), dshape(ndim,nnodemax),
     2 xjac(ndim,ndim),xjaci(ndim,ndim), bmat(nnodemax*ndim), 
     3 statevLocal(nsvint),stress(ntens), ddsdde(ntens, ntens),
     4 stran(ntens), dstran(ntens), wght(ninpt)
c
      dimension predef_loc(npredf),dpredef_loc(npredf),
     1     defGrad(3,3),utmp(3),xdu(3),stiff_p(3,3),force_p(3)
      dimension coord24(2,4),coords_ip(3)
      data  coord24 /dmone, dmone,
     2                 one, dmone,
     3                 one,   one,
     4               dmone,   one/
c
      data wght /one, one, one, one/
c
c*************************************************************
c
c     U1 = first-order, plane strain, full integration
c
c     State variables: each integration point has nsvint SDVs
c
c       isvinc=(npt-1)*nsvint    ... integration point counter
c       statev(1+isvinc        ) ... strain
c
c*************************************************************
      if (lflags(3).eq.4) then
        do i=1, ndofel
          do j=1, ndofel
            amatrx(i,j) = zero
          end do
          amatrx(i,i) = one
        end do
        goto 999
      end if
c
c     PRELIMINARIES
c
      pnewdtLocal = pnewdt
      if(jtype .ne. 1) then
        write(7,*)'Incorrect element type'
        call xit
      endif 
      if(nsvars .lt. ninpt*nsvint) then
        write(7,*)'Increase the number of SDVs to', ninpt*nsvint
        call xit
      endif 
      thickness = 0.1d0
c
c     INITIALIZE RHS AND LHS
c
      do k1=1, ndof*nnode
        rhs(k1, 1)= zero
        do k2=1, ndof*nnode 
          amatrx(k1, k2)= zero
        end do
      end do
c
c     LOOP OVER INTEGRATION POINTS
c
      do kintk = 1, ninpt
c
c       EVALUATE SHAPE FUNCTIONS AND THEIR DERIVATIVES
c
c       determine (g,h)
c
        g = coord24(1,kintk)*gaussCoord
        h = coord24(2,kintk)*gaussCoord
c
c       shape functions
        shape(1) = (one - g)*(one - h)/four;
        shape(2) = (one + g)*(one - h)/four;
        shape(3) = (one + g)*(one + h)/four;
        shape(4) = (one - g)*(one + h)/four;
c
c       derivative d(Ni)/d(g)
        dshape(1,1) = -(one - h)/four;
        dshape(1,2) =  (one - h)/four;
        dshape(1,3) =  (one + h)/four;
        dshape(1,4) = -(one + h)/four;
c
c       derivative d(Ni)/d(h)
        dshape(2,1) = -(one - g)/four;
        dshape(2,2) = -(one + g)/four;
        dshape(2,3) =  (one + g)/four;
        dshape(2,4) =  (one - g)/four;
c
c       compute coordinates at the integration point
c
        do k1=1, 3
          coords_ip(k1) = zero
        end do
        do k1=1,nnode
          do k2=1,mcrd
            coords_ip(k2)=coords_ip(k2)+shape(k1)*coords(k2,k1)
          end do
        end do       
c
c       INTERPOLATE FIELD VARIABLES
c
        if(npredf.gt.0) then

          do k1=1,npredf
            predef_loc(k1) = zero
            dpredef_loc(k1) = zero
            do k2=1,nnode
              predef_loc(k1) =  predef_loc(k1)+ (predef(1,k1,k2)-predef(2,k1,k2))*shape(k2)
             dpredef_loc(k1) =  dpredef_loc(k1)+predef(2,k1,k2)*shape(k2)
            end do
          end do
        end if
c
c       FORM B-MATRIX
c
        djac = one
c
        do i = 1, ndim
          do j = 1, ndim
            xjac(i,j)  = zero
            xjaci(i,j) = zero
          end do
        end do
c     
        do inod= 1, nnode
          do idim = 1, ndim
            do jdim = 1, ndim
              xjac(jdim,idim) = xjac(jdim,idim) + 
     1             dshape(jdim,inod)*coords(idim,inod)
            end do
          end do 
        end do
        djac = xjac(1,1)*xjac(2,2) - xjac(1,2)*xjac(2,1)
        if (djac .gt. zero) then
        ! jacobian is positive - o.k.
          xjaci(1,1) =  xjac(2,2)/djac
          xjaci(2,2) =  xjac(1,1)/djac
          xjaci(1,2) = -xjac(1,2)/djac
          xjaci(2,1) = -xjac(2,1)/djac
        else
          ! negative or zero jacobian
          write(7,*)'WARNING: element',jelem,'has neg. 
     1         Jacobian'
          pnewdt = fourth
        endif
        

        if (pnewdt .lt. pnewdtLocal) pnewdtLocal = pnewdt
c
        do i = 1, nnode*ndim
          bmat(i) = zero
        end do

        do inod = 1, nnode
          do ider = 1, ndim
            do idim = 1, ndim
              irow = idim + (inod - 1)*ndim
              bmat(irow) = bmat(irow) + 
     1             xjaci(idim,ider)*dshape(ider,inod)      
            end do
          end do
        end do 

c
c       CALCULATE INCREMENTAL STRAINS
c 
        do i = 1, ntens
          dstran(i) = zero
        end do
        !
        ! set deformation gradient  to Identity matrix
        do k1=1,3
          do k2=1,3
            defGrad(k1,k2) = zero
          end do
          defGrad(k1,k1) = one
        end do
c
c       COMPUTE INCREMENTAL STRAINS
c
        do nodi = 1, nnode
           
          incr_row = (nodi - 1)*ndof

          do i = 1, ndof
            xdu(i)= du(i + incr_row,1)
            utmp(i) = u(i + incr_row)
          end do

          dNidx = bmat(1 + (nodi-1)*ndim)
          dNidy = bmat(2 + (nodi-1)*ndim)

          dstran(1) = dstran(1) + dNidx*xdu(1)
          dstran(2) = dstran(2) + dNidy*xdu(2)
          dstran(4) = dstran(4) + 
     1         dNidy*xdu(1) + 
     2         dNidx*xdu(2)  

c        deformation gradient

          defGrad(1,1) = defGrad(1,1) + dNidx*utmp(1)
          defGrad(1,2) = defGrad(1,2) + dNidy*utmp(1)
          defGrad(2,1) = defGrad(2,1) + dNidx*utmp(2)
          defGrad(2,2) = defGrad(2,2) + dNidy*utmp(2)
        end do

c        
c       CALL CONSTITUTIVE ROUTINE
c

        isvinc= (kintk-1)*nsvint  ! integration point increment
c
c       prepare arrays for entry into material routines
c
        do i = 1, nsvint
          statevLocal(i)=svars(i+isvinc)
        end do
c
c       state variables
c
!DEC$ NOVECTOR
        do k1=1,ntens
           stran(k1) = statevLocal(k1)
           stress(k1) = zero
        end do
c
        do i=1, ntens
!DEC$ NOVECTOR
          do j=1, ntens
            ddsdde(i,j) = zero
          end do
          ddsdde(i,j) = one
        enddo
c
c       compute characteristic element length
c
        celent = sqrt(djac*dble(ninpt))
        dvmat  = djac*thickness
c
        dvdv0 = one
        call material_lib_mech(materiallib,stress,ddsdde,
     1       stran,dstran,kintk,dvdv0,dvmat,defGrad,
     2       predef_loc,dpredef_loc,npredf,celent,coords_ip)
c
        do k1=1,ntens
           statevLocal(k1) = stran(k1) + dstran(k1)
        end do
c
        isvinc= (kintk-1)*nsvint  ! integration point increment
c
c       update element state variables 
c
        do i = 1, nsvint
          svars(i+isvinc)=statevLocal(i)
        end do
c        
c       form stiffness matrix and internal force vector
c
        dNjdx = zero
        dNjdy = zero
        do i = 1, ndof*nnode
          force(i) = zero
          do j = 1, ndof*nnode
            stiff(j,i) = zero
          end do
        end do

        dvol= wght(kintk)*djac
        do nodj = 1, nnode

          incr_col = (nodj - 1)*ndof

          dNjdx = bmat(1+(nodj-1)*ndim)
          dNjdy = bmat(2+(nodj-1)*ndim)
          force_p(1) = dNjdx*stress(1) + dNjdy*stress(4)
          force_p(2) = dNjdy*stress(2) + dNjdx*stress(4)
          do jdof = 1, ndof

            jcol = jdof + incr_col

            force(jcol) = force(jcol) + orce_p(jdof)*dvol
            
          end do

          do nodi = 1, nnode

            incr_row = (nodi -1)*ndof

            dNidx = bmat(1+(nodi-1)*ndim)
            dNidy = bmat(2+(nodi-1)*ndim)
            stiff_p(1,1) = dNidx*ddsdde(1,1)*dNjdx + dNidy*ddsdde(4,4)*dNjdy + dNidx*ddsdde(1,4)*dNjdy + dNidy*ddsdde(4,1)*dNjdx
            
            stiff_p(1,2) = dNidx*ddsdde(1,2)*dNjdy + dNidy*ddsdde(4,4)*dNjdx + dNidx*ddsdde(1,4)*dNjdx + dNidy*ddsdde(4,2)*dNjdy
              
            stiff_p(2,1) = dNidy*ddsdde(2,1)*dNjdx + dNidx*ddsdde(4,4)*dNjdy + dNidy*ddsdde(2,4)*dNjdy + dNidx*ddsdde(4,1)*dNjdx

            stiff_p(2,2) = dNidy*ddsdde(2,2)*dNjdy + dNidx*ddsdde(4,4)*dNjdx + dNidy*ddsdde(2,4)*dNjdx + dNidx*ddsdde(4,2)*dNjdy
              
            do jdof = 1, ndof
              icol = jdof + incr_col
              do idof = 1, ndof
                irow = idof + incr_row
                stiff(irow,icol) = stiff(irow,icol) + stiff_p(idof,jdof)*dvol
              end do
            end do
          end do
        end do

c
c       assemble rhs and lhs
c
        do k1=1, ndof*nnode
            rhs(k1, 1) = rhs(k1, 1) - force(k1) 
          do k2=1, ndof*nnode
            amatrx(k1, k2) = amatrx(k1, k2) + stiff(k1,k2)
          end do
        end do
      end do       ! end loop on material integration points
      pnewdt = pnewdtLocal
c
 999  continue
c
      return
      end
