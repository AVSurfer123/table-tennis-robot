#!/usr/bin/env julia

#state matrix = S in kernel space = Φ
#goal matrix = Γ
#cost matrix = C
#scaling parameter = λ

#module crkr_sparse
using Distributions
using DataStructures
using Distances
#using PyCall
#@pyimport numpy as np


abstract KdNode

type KdItem <: KdNode
  val::Array{Float64,1}
  left::KdNode
  right::KdNode
  level::Int64 #level in [0,1,...]
end

type KdEmpty <: KdNode
end

type Kd
  root::KdNode
end

item(x::Array{Float64,1}, level::Int64)=KdItem(x,KdEmpty(),KdEmpty(),level)::KdItem

along_dim(level::Int64,dims::Int64) = (level%dims)+1

function add_item(t::Kd, val::Array{Float64,1})
  cnode = t.root
  prev=cnode
  branch = 0

  finished = false
  l=0
  while !finished
    d = along_dim(l, size(val)[1])
    if typeof(cnode) == KdEmpty
      cnode = item(val,l)
      if branch == 1
        prev.right=cnode
      elseif branch == -1
        prev.left=cnode
      else
        t.root = cnode
      end
      finished=true
    else
      l += 1
      prev=cnode
      if val[d] > cnode.val[d]
        cnode = cnode.right
        branch = 1
      else
        cnode = cnode.left
        branch = -1
      end
    end
  end
end

distance(s1,s2) = norm(s1-s2)::Float64


function rquery(n::KdNode, val::Array{Float64,1}, vBest::Array{Float64,1}, dBest::Float64)
  #println("Checking:", n.val)
  if typeof(n) == KdEmpty
    return (vBest,dBest)
  end
  d = along_dim(n.level, size(val)[1])
  dist = distance(val, n.val)
  if typeof(n.left) != KdEmpty && val[d] <= n.val[d] #&& distance(n.val[d], val[d]) <= dBest
      (vBest,dBest)=rquery(n.left, val, vBest, dBest)
  end
  if typeof(n.right) != KdEmpty && val[d] >= n.val[d] && distance(n.val[d], val[d]) <= dBest
       (vBest,dBest)=rquery(n.right, val, vBest, dBest)
  end
  if dist < dBest
    dBest = dist
    vBest = n.val
    #println("New best:", dBest)
  end
  return (vBest,dBest)
end


function linFindClos(val::Array{Float64,1},vals::Array{Float64,2})
  best = vals[:,1]
  best_d = distance(vals[:,1], val)
  for i=1:size(vals)[2]
    d = distance(vals[:,i], val)
    if d < best_d
      best_d = d
      best = vals[:,i]
    end
  end
  return best
end

#export add_sample, predict, RegData, initRegData, test, k, draw_normal, defk, rebuild_K, get_iter

type RegData
    iter::Int  # current iteration (number of collected regression samples)
    λ::Float64  # Scalling parameter exploration/exploitation tradeoff
    MAX_ITER::Int  # Maximum number of iterations
    S::Array{Float64, 2}
    Γs::Array{Array{Float64, 1},1}
    C::Array{Float64, 2}
    Ks::Array{Array{Float64, 2},1}
    #cKs::Array{Base.LinAlg.Cholesky{Float64,Array{Float64,2}},1}
    cKs::Array{Base.LinAlg.Cholesky{Float64,SubArray{Float64,2,Array{Float64,2},Tuple{UnitRange{Int64},UnitRange{Int64}},1}},1}
    σfs::Array{Float64, 1}
    ls::Array{Float64, 2}
    qs::Collections.PriorityQueue{Tuple{Array{Float64,1},Array{Float64,1},Float64},Float64}
end

function initRegData(λ::Float64, MAX_ITER::Int, sdim=1::Int, γdim=1::Int, σfs=[1.0]::Array{Float64,1}, ls=[[0.5]]::Array{Float64,2})
    S = zeros(MAX_ITER,sdim)::Array{Float64,2}
    Γs = [zeros(MAX_ITER)::Array{Float64,1} for i=1:γdim] ::Array{Array{Float64, 1},1}
    C = zeros(MAX_ITER,MAX_ITER)::Array{Float64, 2}
    a=rand(MAX_ITER,MAX_ITER)
    Ks = [a'a::Array{Float64,2} for i=1:γdim] ::Array{Array{Float64, 2},1}
    cKs = [cholfact!(sub(Ks[i],1:MAX_ITER,1:MAX_ITER))::Base.LinAlg.Cholesky{Float64,SubArray{Float64,2,Array{Float64,2},Tuple{UnitRange{Int64},UnitRange{Int64}},1}} for i=1:γdim]::Array{Base.LinAlg.Cholesky{Float64,SubArray{Float64,2,Array{Float64,2},Tuple{UnitRange{Int64},UnitRange{Int64}},1}},1}
    qs=Collections.PriorityQueue(Tuple{Array{Float64,1},Array{Float64,1},Float64},Float64)
    if size(ls) != (γdim,sdim)
      throw(ErrorException("incorrect ls dimensions (y,x)"))
    elseif size(σfs) != (γdim,)
      throw(ErrorException("incorrect σfs dimensions (y,x)"))
    end
    return RegData(0, λ, MAX_ITER, S, Γs, C, Ks, cKs, σfs, ls, qs)
end

#σf = 1.0#1.1251 #parameter of the squared exponential kernel
#l =  0.5#0.90441 #parameter of the squared exponential kernel
#k(s1,s2) = σf^2 *exp(-sum((s1-s2).^2)/(2*l^2))
#o**2 * np.exp(-0.5*sum((s1-s2)**2 / h**2))
#k(s1,s2,σf,l) = σf^2*exp(-(s1-s2)'*(l.^(-2))(s1-s2)/2) #σf^2 *exp(-0.5*sum((s1-s2).^2 ./l.^2)) #SE-ARD
k(s1,s2,σf,l) = σf*exp(-0.5*evaluate(WeightedSqEuclidean(1.0./(l)), s1, s2))


function kϕ(sj, S, σf, l)
#  println(size(sj),size(S[1,:]),size(σf), size(l))
        out=zeros(size(S)[1],1)
        for i=1:size(S,1)
            out[i,:]=k(sj,vec(S[i,:]),σf, l) # transpose since size(S[i,:])=(1,2)
        end
        #println(out)
    return out
end

function exhaustiveS(val::Array{Float64,1} ,ps::Array{Array{Float64,1},1}, best::Float64)
  v=val[:]::Array{Float64,1}
  for p in ps
    if distance(val,p) < best
      v[:] = p[:]
      best = distance(val,p)::Float64
    end
  end
  return (v,best)
end

function get_new_mat(d::RegData)
  #result=Collections.PriorityQueue(Array{Float64,1},Float64)
  result=Collections.PriorityQueue(Tuple{Array{Float64,1},Array{Float64,1},Float64},Float64)
  val=Collections.dequeue!(d.qs)

  #exhaustiveS
  Collections.enqueue!(result, val, 1.0/1.0e9)  #add_item(tree, val[1])
  ls=Array(Array{Float64,1},1)::Array{Array{Float64,1},1}
  ls[1]=val[1]
  #push!(ls, val[1])
  while !isempty(d.qs)
      val=Collections.dequeue!(d.qs)
      #(q,ri)=rquery(tree.root, val[1], tree.root.val, 1.0e9)
      (q,ri) = exhaustiveS(val[1], ls, 1.0e9)
      Collections.enqueue!(result, val, 1.0/ri)
      #add_item(tree, val[1])
      push!(ls, val[1])
  end

  #=
  tree = Kd(KdEmpty())
  add_item(tree, val[1])
  Collections.enqueue!(result, val, 1.0/1.0e9)
  while !isempty(d.qs)
    val=Collections.dequeue!(d.qs)
    (q,ri)=rquery(tree.root, val[1], tree.root.val, 1.0e9)
    Collections.enqueue!(result, val, 1.0/ri)
    add_item(tree, val[1])
  end
  =#

  #using select! would be faster

  i=0::Int64
  while !isempty(result)
    val=Collections.dequeue!(result)
    Collections.enqueue!(d.qs,val,val[3])
    i+=1
    if i<=size(d.S)[1]
      d.S[i,:]=val[1]
      d.C[i,i]=val[3]
      for n=1:size(d.Γs)[1]
        d.Γs[n][i]=val[2][n]
      end
    end
  end
  println("total samples: ",i)
end

function add_sample!(d::RegData, sj::Array{Float64,1}, γj::Array{Float64,1}, cj::Float64)
    Collections.enqueue!(d.qs,(sj,γj,cj),cj)
    if d.iter >= d.MAX_ITER
      get_new_mat(d)
      rebuild_Ks(d)
      return true
    else
        d.iter += 1
        j=d.iter
        d.S[j,:] = sj[:]
        for ns=1:size(d.Γs)[1]
          d.Γs[ns][j] = γj[ns]
        end
        d.C[j, j] = cj
        #updateKs(d,d.iter)
        rebuild_Ks(d)
        return true
    end
    return false
end

function rebuild_Ks(d::RegData)
  for n=1:size(d.Γs)[1]
    pairwise!(sub(d.Ks[n],1:d.iter,1:d.iter), WeightedSqEuclidean(1.0./(d.ls[n,:]')), d.S[1:d.iter,:]')
    d.Ks[n][1:d.iter,1:d.iter] = d.σfs[n]*exp(-0.5*sub(d.Ks[n],1:d.iter,1:d.iter))[:]
    #d.Ks[n][1:d.iter,1:d.iter] = d.σfs[n]*exp(pairwise(WeightedSqEuclidean(1.0./(d.ls[n,:]')), d.S[1:d.iter,:]'))
    for i=1:d.iter
        #=for j=i:d.iter
            d.Ks[n][i,j]=k(sub(d.S,i,:),sub(d.S,j,:), d.σfs[n], d.ls[n,:])[1]::Float64
            d.Ks[n][j,i]=d.Ks[n][i,j]::Float64
        end=#
        d.Ks[n][i,i] = d.Ks[n][i,i] + d.λ*d.C[i,i]::Float64
    end
    if !isposdef(d.Ks[n][1:d.iter,1:d.iter]) println((n,d.iter,d.σfs[n], d.ls[n,:])) end
    d.cKs[n] = cholfact!(sub(d.Ks[n],1:d.iter,1:d.iter))
  end
end

function predict(sj::Array{Float64,1}, d::RegData)
    mγj=zeros(size(d.Γs))
    σ2=zeros(size(d.Γs))

#    @parallel for n=1:size(d.Γs)[1]
    for n=1:size(d.Γs)[1]
      _kϕ=kϕ(sj, sub(d.S,1:d.iter,:), d.σfs[n], vec(d.ls[n,:])) #optimization since value is reused
      mγj[n] = (_kϕ' * (d.cKs[n] \ sub(d.Γs[n], 1:d.iter,:)) )[1]::Float64  # (N*j) (j*N)
      σ2[n] =  ( k(sj, sj, d.σfs[n], d.ls[n,:] ) - _kϕ'* (d.cKs[n] \ _kϕ) )[1] ::Float64
      #The Regression needs to be without costs (λC)
      #σ2[n] = (1-exp(-((_kϕ' * (d.cKs[n] \ (d.λ*diag(d.C))[1:d.iter]))[1])^2)) ::Float64
    end
    return mγj, σ2
end

#draw_normal(N::Tuple{Array{Float64,1}, Float64}) = rand(MvNormal(N[1], N[2]*eye(size(N[1])[1]))) ::Array{Float64, 1}
draw_normal(N::Tuple{Array{Float64,1}, Float64}) = rand(MvNormal(N[1], sqrt(N[2])*eye(size(N[1])[1]))) ::Array{Float64, 1}
draw_normal(N::Tuple{Array{Float64,1}, Array{Float64,1}}) = rand(MvNormal(N[1], diagm(sqrt(N[2])))) ::Array{Float64, 1}

tri(x)=asin(sin(x))
tricost(s,y)=10*(tri(s)-y)^2

function remdiag(A::Array{Float64,2})
    sizeB=(size(A)[1], size(A)[2]-1)
    B=zeros(sizeB)
    inds = trues(size(A)[1])
    for i=1:sizeB[1]
        inds[i]=false
        B[i,:] = A[i,inds]
        inds[i]=true
    end
    return B
end

cholupdate(A,B)=cholfact(A'A+B'B).UL #Numerically unstable

function ch_update(ck, j)
    ck2 = cholfact(eye(ck.UL[1:end-1,1:end-1]))
    ck2.UL[1:j-1, 1:j-1] = ck.UL[1:j-1, 1:j-1]
    ck2.UL[1:j-1, j:end] = ck.UL[1:j-1, j+1:end]
    ck2.UL[j:end, 1:j-1] = 0
    ck2.UL[j:end, j:end] = cholupdate(ck.UL[j+1:end, j+1:end],ck.UL[j, j+1:end])
    ck2
end

get_iter(d::RegData)=d.iter

function test()

    MAX_ITER=100
    #rdata=initRegData(1.0, 100)
    rdata=initRegData(1.0, MAX_ITER, 1, 1, [1.0], 0.5*ones(1,1))
    srand(1234)
    s0 = Array(Float64, 1)
    γ0 = Array(Float64, 1)

    s0[1] = 0.1
    γ0[1] = 2.0
    c0=tricost(s0[1],γ0[1])::Float64

    add_sample!(rdata, s0, γ0, c0)
    cost_hist = zeros(MAX_ITER)

    test_set=linrange(0,1,10)

    for j = 1:MAX_ITER-1
        sj = rand((1,1))::Array{Float64,2}
        if j%10 == 0#run test set
            c = zeros(10)
            for jj=1:10
                sj[:] = test_set[jj]
                s,o=predict(vec(sj), rdata)
                c[jj]=tricost(sj[1],s[1])
            end
            cost_hist[j/10]=mean(c)
            println(mean(c))
        end
        if j < MAX_ITER-1
            sj = rand(1)::Array{Float64,1}
            s=draw_normal(predict(sj, rdata))
            #[tricost(s[1],s[1])]
            c=tricost(sj[1],s[1])
            add_sample!(rdata, [sj[1]], [s[1]], c)
        else
            predict(vec(sj), rdata)
            break
        end
    end
    return rdata
end
#@time test()
#end #end module
