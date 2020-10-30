using RobotOS
@rosimport arm_moveit_trajectory.msg: ball_state
@rosimport optitrack_sim_tools.msg: ModelState, ModelStates, SystemState, findResultAction, findResultGoal
rostypegen()
using arm_moveit_trajectory.msg
using optitrack_sim_tools.msg
using PyCall
using DataFrames

launch_gen() = [-0.5, -2.0, 1.8, 0.0, 3.0, 2.5] + (0.5*[1.0,0.0,1.0,1.0,1.0,1.0].*(rand(6)-0.5))
#TEST_SET=[launch_gen() for i in range(20)]

function initSaveSamples()
  df = DataFrame(sj=Tuple{Array{Float64,1}}[], gj = Tuple{Array{Float64,1}}[], hit = Bool[], cj = Float64[], endEffectHitTime = Tuple{Array{Float64,1}}[], ballHitTime = Tuple{Array{Float64,1}}[], tablePos = Tuple{Array{Float64,1}}[])
  dfr = DataFrame()
  try dfr=readtable("data.csv")
  catch SystemError
    return df
  end
  for i=1:size(dfr)[1]
    push!(df, [eval(parse(dfr[:sj][i])); eval(parse(dfr[:gj][i])); dfr[:hit][i]; dfr[:cj][i]; eval(parse(dfr[:endEffectHitTime][i])); eval(parse(dfr[:ballHitTime][i])); eval(parse(dfr[:tablePos][i]))])
  end
  return df
end

function genTestSet(seed::Int, nTest::Int)
  srand(seed)
  return hcat([launch_gen() for i=1:nTest]...)
end

#=
mean([launch_gen()[6] for i=1:100])
mean(genTestSet()[6,:])
=#

#from 2crkr_node.py
#genTestSet() = [[-0.53402723, -2.,          1.67456831,  0.06033063,  2.86507603,  2.34334034] [-0.38832639, -2.,          1.72626902,  0.06840083,  2.78256741,  2.65071708] [-0.70541687, -2.,          1.96393834, -0.14070398,  2.783651,    2.5029549 ] [-0.37164265, -2.,          1.75272156,  0.08464947,  2.96988838,  2.5760203 ] [ -6.08903004e-01,  -2.00000000e+00,   1.79743065e+00,   2.32585285e-03,   3.20261191e+00,   2.70034180e+00] [-0.55774762, -2.,          1.82636854,  0.12989468,  2.94347483,  2.28665602] [-0.30268666, -2.,          1.91613943, -0.20153568,  3.09921404,  2.29800599] [-0.48263445, -2.,          1.65688158,  0.06321492,  2.95424919,  2.47217659] [-0.7538667,  -2.,          1.7053004,  -0.00296396,  2.90218856,  2.57651602] [-0.49508543, -2.,          1.93909789, -0.11358313,  2.96801331,  2.43700063] [-0.56466745, -2.,          1.70819444,  0.05191406,  2.96302901,  2.35561168] [-0.79231403, -2.,          1.88515096, -0.08751187,  2.93803589,  2.62161405] [-0.37347343, -2.,          1.60970914, -0.11986723,  2.89580612,  2.68556436] [-0.78269792, -2.,          1.84333353,  0.05652952,  3.05625366,  2.45498574] [-0.3228492,  -2.,          1.81844467,  0.22965787,  2.79485317,  2.44432072] [-0.30880802, -2.,          1.54078333,  0.24980452,  2.84182373,  2.35232654] [-0.62667653, -2.,          1.74135596,  0.02886422,  2.98623682,  2.72874698] [-0.3968861,  -2.,          1.71101507,  0.05714658,  2.78662118,  2.7024074 ] [-0.74977833, -2.,          1.84733484,  0.22069184,  3.21972058,  2.74787844] [-0.53122866, -2.,          1.77818206,  0.0136137,   2.96434966,  2.69453918] ]::Array{Float64,2}

type TrainState # type that selects if it is a training or a test sample
  isTraining::Bool
  k::Int64
  testN::Int64
  trainN::Int64
end

function updateS!(s::TrainState) # function to update the state
  current = (s.isTraining,s.k)
  s.k += 1
  if (!s.isTraining && s.k > s.testN) || (s.isTraining && s.k>s.trainN)
    s.k=1
    s.isTraining = !s.isTraining
  end
  return current
end

@pyimport pyximport
unshift!(PyVector(pyimport("sys")["path"]), "")
pyximport.install()
@pyimport numpy as np
function load_qp_cython()
  dmp_test_path = np.load("dmp_artificial_demonstration_6dof_x5.npy")
  omp = cython_csolver.OptUnconstrained(dmp_test_path, dt=0.001, n_bfs=60)
  return  omp
end

function logProgress!(s::TrainState, hit::Bool, cj::Float64, cjs::Array{Float64,1}, hts::Array{Float64,1}) # function to update the state
  #current = (s.isTraining,s.k)
  #s.k += 1
  if s.k==1 && s.isTraining && cjs[end] != 0.0 #hack to avoid repeating
    push!(hts,0.0)
    push!(cjs,0.0)
    #s.k=1
    #s.isTraining = !s.isTraining
  elseif !s.isTraining
    cjs[end] += cj/s.testN
    if hit
      hts[end] += 1
    end
  end
  #return current
end

function cost_fun(target, ground_hit, ball_hit_time, racket_hit_time)
    const a=0.2
    const k=-(1/(2a^2))::Float64
    distance_cost=0.0#sqeuclidean(ground_hit[1:2],target[1:2])
    #hit_cost = (100*sqeuclidean(ball_hit_time[1:3],racket_hit_time[1:3]))^2
    #1-exp(-(1/(2a^2)).*abs(x).^2)
    hit_cost = 1.0 - exp(k*sqeuclidean(ball_hit_time[1:3],racket_hit_time[1:3]))

    #println(ground_hit[1:2] - target)
    #println(ball_hit_time - racket_hit_time)^2)
    #hit_cost = 0.0
    #hit_cost = norm(ball_hit_time - racket_hit_time)^2
    #accel_cost = np.linalg.norm(np.diff(dmp_path, 2))**2
    #hit_cost = (xpath[t_hit_sample_n]-dmp_path[t_hit_sample_n])**2
    #print('ball:{} \n racket:{}\n'.format(ball_hit_time,racket_hit_time))
    return (distance_cost + hit_cost) 
  end

include("/home/vislab/indigo_ws/julia/CrkrJL/crkr_anms_v2.jl")

unshift!(PyVector(pyimport("sys")["path"]), "")
@pyimport publish_trajectory
@pyimport actionlib
@pyimport control_msgs.msg as cmsg#: JointTrajectoryAction, FollowJointTrajectoryAction
@pyimport optitrack_sim_tools.msg as optmsg
@pyimport openravepy as opr
@pyimport rospy
@pyimport tf

#needs the node to be initialized
function getRtoRef()
  listener = tf.TransformListener()
  rospy.rostime["wallsleep"](0.5) # Important simulates a spinOnce to allow the thread to continue
  (trans, rot) = listener["lookupTransform"]("optitrack_frame", "world", rospy.Time(0))
  tros = tf.TransformerROS()
  R_to_ref = tros["fromTranslationRotation"](trans, rot)
  return R_to_ref
end

function getOpenraveRobot()
  env = opr.Environment()
  kinbody = env["ReadRobotXMLFile"]("/home/vislab/indigo_ws/julia/CrkrJL/robots/x6.xml")
  env["Add"](kinbody)
  robot = env["GetRobots"]()[1]
  robot["SetActiveManipulator"]("arm")

    # generate the ik solver
  ikmodel = opr.databases["inversekinematics"]["InverseKinematicsModel"](robot,
                                                                     iktype=opr.IkParameterization["Type"]["TranslationDirection5D"])
                                                                     #if not ikmodel.load():
                                                                     #    ikmodel.autogenerate()
  return robot, ikmodel
  end

function launchBs!(bs::optitrack_sim_tools.msg.ModelState, isTraining::Bool, counter::Int64, TEST_SET::Array{Float64,2})
    #isTraining ? (bs.pose.position.x, bs.pose.position.y, bs.pose.position.z, bs.twist.linear.x, bs.twist.linear.y, bs.twist.linear.z) =  launch_gen():
    #(bs.pose.position.x, bs.pose.position.y, bs.pose.position.z, bs.twist.linear.x, bs.twist.linear.y, bs.twist.linear.z) = TEST_SET[:,counter]
    isTraining ? setBs!(bs, launch_gen(),"ping_pong_ball") : setBs!(bs, TEST_SET[:,counter],"ping_pong_ball")
    return bs
end

function getGj(isTraining::Bool, crkr_inst::RegData, sj::Array{Float64,1})
    isTraining ? gj = trunc_norm(predict(sj, crkr_inst)): gj = predict(sj, crkr_inst)[1] ::Array{Float64,1}
    return gj
end

#model=
#"ping_pong_ball"
#"blue_ping_pong_ball" tuple(result.ball_thit[:3])
#"green_ping_pong_ball" dir_kin_hit_time
#"red_ping_pong_ball" tuple(result.ball_table[:3])
function setBs!(bs::ModelState, poseTwist::Array{Float64,1}, model::ASCIIString)
  (bs.pose.position.x, bs.pose.position.y, bs.pose.position.z, bs.twist.linear.x, bs.twist.linear.y, bs.twist.linear.z) = poseTwist
  bs.reference_frame = "world"::ASCIIString
  bs.model_name = model::ASCIIString
  return bs::ModelState
end

function setBs!(bs::ModelState, poseTwist::Array{Float64,1}, model::ASCIIString, zs6::Array{Float64,1})
  zs6[1:3]=poseTwist[1:3]::Array{Float64,1}
  return setBs!(bs, zs6, model)::ModelState
end

function saturate!(mu::Array{Float64,1}, mmin::Array{Float64,1}, mmax::Array{Float64,1})
  for (k,m)=enumerate(mu)
    if mu[k] > mmax[k]
      mu[k]=mmax[k]
    elseif mu[k] < mmin[k]
      mu[k]=mmin[k]
    end
  end
end

function trunc_norm(ts::Tuple{Array{Float64,1}, Float64})
  mu=ts[1]
  sigma=sqrt(ts[2])
  jmax = [2.53, 2.0071, 2.007, 1.83, 1.57, 1.57, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.7]
  jmin = [-2.53, -2.181, -2.007, -1.83, -1.57, -1.57, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, 0.3]
  saturate!(mu,jmin,jmax)
  return [rand(Truncated(Normal(mu[i], sigma), jmin[i], jmax[i]))::Float64 for i=1:length(mu)]::Array{Float64,1}
end

function trunc_norm(ts::Tuple{Array{Float64,1}, Array{Float64,1}})
  mu=ts[1]
  sigma=sqrt(ts[2])
  jmax = [2.53, 2.0071, 2.007, 1.83, 1.57, 1.57, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 1.7]
  jmin = [-2.53, -2.181, -2.007, -1.83, -1.57, -1.57, -3.0, -3.0, -3.0, -3.0, -3.0, -3.0, 0.3]
  saturate!(mu,jmin,jmax)
  println(sigma)
  println(mu)
  return [rand(Truncated(Normal(mu[i], sigma[i]), jmin[i], jmax[i]))::Float64 for i=1:length(mu)]::Array{Float64,1}
end

function pub_markers(pub_ms::Publisher, bs::ModelState, res_bthit::Array{Float64,1}, dir_kin_hit_time::Array{Float64,1}, res_bt::Array{Float64,1}, zs6::Array{Float64,1})
      setBs!(bs, res_bthit[1:3], "blue_ping_pong_ball", zs6)
      publish(pub_ms,bs)
      setBs!(bs, dir_kin_hit_time[1:3], "green_ping_pong_ball", zs6)
      publish(pub_ms,bs)
      setBs!(bs, res_bt[1:3], "red_ping_pong_ball", zs6)
      publish(pub_ms,bs)
end

function load_training!(crkr_inst::RegData)
  tqs = readdlm("qs.txt")
  for i=1:size(tqs)[1]
      Collections.enqueue!(crkr_inst.qs, eval(parse(tqs[i,1])),tqs[i,2])
  end
  k =  size(tqs)[1] #round(Int64, readdlm("iter.txt")[1])::Int64
  k > size(crkr_inst.S)[1] ? k = size(crkr_inst.S)[1] : k=k
  crkr_inst.iter = k

    val=Collections.dequeue!(crkr_inst.qs)
    add_sample!(crkr_inst, val[1], val[2], val[3]) # should deal with everything including rebuilding th matrix
    #crkr_inst.iter=k
    #crkr_inst.C[1:k,1:k] = readdlm("C.txt")[1:k,1:k]::Array{Float64,2}
    #crkr_inst.S[1:k,:] = readdlm("S.txt")[1:k,:]::Array{Float64,2}
    #crkr_inst.Γ[1:k,:] = readdlm("G.txt")[1:k,:]::Array{Float64,2}
    #rebuild_K(crkr_inst)
end

function load_demos!(crkr_inst::RegData, d::Int64)
  tqs = readdlm("qs.txt")
  for i=1:size(tqs)[1]
      Collections.enqueue!(crkr_inst.qs, eval(parse(tqs[i,1])),tqs[i,2])
  end
  k =  d#size(tqs)[1] #round(Int64, readdlm("iter.txt")[1])::Int64
  #k > size(crkr_inst.S)[1] ? k = size(crkr_inst.S)[1] : k=k
  crkr_inst.iter = k

    val=Collections.dequeue!(crkr_inst.qs)
    add_sample!(crkr_inst, val[1], val[2], val[3]) # should deal with everything including rebuilding th matrix
end

function load_training!(crkr_inst::RegData, n::Int64)
  tqs = readdlm("qs.txt")
  for i=1:size(tqs)[1]
      val=eval(parse(tqs[i,1]))
      Collections.enqueue!(crkr_inst.qs, (val[1],[val[2][n]],val[3]),tqs[i,2])
  end
  k =  size(tqs)[1] #round(Int64, readdlm("iter.txt")[1])::Int64
  k > size(crkr_inst.S)[1] ? k = size(crkr_inst.S)[1] : k=k
  crkr_inst.iter = k

  val=Collections.dequeue!(crkr_inst.qs)
  add_sample!(crkr_inst, val[1], val[2], val[3]) # should deal with everything including rebuilding th matrix
end

function save_training(crkr_inst::RegData, df::DataFrame)
  println("saving")
  writedlm("qs.txt", crkr_inst.qs)
  writedlm("C.txt", crkr_inst.C)
  writedlm("S.txt", crkr_inst.S)
  writedlm("G.txt", crkr_inst.Γs)
  writedlm("iter.txt", crkr_inst.iter)
  writetable("data.csv", df)
end

#check_success(res_bt::Array{Float64,1}) = (res_bt[2] < 0.0 && res_bt[5] < 0.0) ? (true,res_bt)::Tuple{Bool,Array{Float64,1}} : (false,[0.0,10.0,0.0,0.0,0.0,0.0])::Tuple{Bool,Array{Float64,1}}
#check_success(res_bt::Array{Float64,1}) = (res_bt[5] <= 1.0) ? (true,res_bt)::Tuple{Bool,Array{Float64,1}} : (false,[0.0,10.0,0.0,0.0,0.0,0.0])::Tuple{Bool,Array{Float64,1}}
check_success(res_bt::Array{Float64,1}) = (res_bt[2] <= -0.05) ? (true,res_bt)::Tuple{Bool,Array{Float64,1}} : (false,[0.0,10.0,0.0,0.0,0.0,0.0])::Tuple{Bool,Array{Float64,1}}

function print_info(hit::Bool, s::TrainState, crkr_inst::RegData, cj::Float64, cjs::Array{Float64,1}, hts::Array{Float64,1})
  hit ? print("hit  "):print("miss ")
  s.isTraining ? print(" training: "): print(" test: ")
  println(s.k," iter:",crkr_inst.iter," c:", cj)
  if s.k == 1
    println(cjs)
    println(hts)
  end
end

function print_info(hit::Bool, crkr_inst::RegData, cj::Float64, cjs::Array{Float64,1}, hts::Array{Float64,1}, filename::ASCIIString)
  f = open(filename,"w")
  write(f,string(ARGS))
  write(f,"\n")
  write(f,string(cjs))
  write(f,"\n")
  write(f,string(hts))
  close(f)
end


function run_node(λ=1.0e-2::Float64, maxIters=500::Int64, n_epochs=50::Int64, testOnly=false::Bool)
  #definitions
  const saveTraining = true ::Bool
  const loadPreviousTraining = false::Bool
  const nLearn = 50 ::Int64#50
  const nTest = 50 ::Int64#50
  const updateAlways = false::Bool
  #const λ = 1.0e-7::Float64
  #const h = 1.0
  #const σ = 1.33

  const TARGET = [0.0, -2.0]::Array{Float64,1}
  const TEST_SET = genTestSet(1234, nTest)::Array{Float64,2}

  const par = [[8.905206075690884,6.857632565373989,9.025013499434122,7.068512119641409,7.461235925710409,9.286537232555325,9.984090746783785],[2.127300373731715,1.2685445610026589,9.025013499434122,7.698998076983106,2.4929191975861755,15.25665424688945,2.4338001577659947],[1.6188404377661088,1.7494929074426921,9.025013499434122,4.402733030808015,3.286987614463831,16.08069538339252,2.3112910650252183],[2.803818997318236,2.453874141011896,9.025013499434122,2.4958442569183967,2.4278998073301596,4.927272000631103,1.6993173072360612],[2.3789036160376926,0.9397668783505886,9.025013499434122,3.352908016254941,2.023065845154516,5.7366614499076265,1.8172497631913536],[0.457926948489661,1.2671389510534605,9.025013499434122,1.6556846158562377,0.5182597304720581,0.9753724887236748,0.5669539036719348],[0.350589341899041,0.6660608896083218,9.025013499434122,1.1825894869840532,1.049419941604214,1.4520771711748452,1.093740737770352],[1.2185069954529866,0.9375417794549018,9.025013499434122,0.49833830121783745,0.8269116733188144,1.2930866400203096,0.3462148062723412],[4.501534969590698,1.3625987856286015,9.025013499434122,1.130020897776927,1.0822437171310726,4.221421240131715,0.9110421428541141],[7.133931006133042,1.2509049813884001,9.025013499434122,1.0626310807693853,0.9950167213915811,2.8506592024299713,3.932801533671008],[8.835967820931195,11.941292857724804,9.025013499434122,1.7504393399837859,2.1314317292720135,3.1323605098098533,7.709101115374043],[10.440113143674404,6.1632912613776,9.025013499434122,2.789768536286044,3.211837895608046,4.91043470078673,11.82564579616937],[4.554317827271898,12.071972119988512,9.025013499434122,10.118459642768045,7.676239880542076,14.02736465165498,8.221118310915658]]
  const σs = par[1:7:91]
  hs = zeros(13,6)
  for i=2:7 hs[:,i-1] = par[i:7:91] end

  crkr_inst = initRegData(λ, maxIters, 6, 13, σs, hs)
  #crkr_inst = initRegData(λ, maxIters, 6, 13, 1.33ones(size(σs)), 0.5ones(size(hs)))

  if loadPreviousTraining
    load_training!(crkr_inst)
  else

  best=[([-0.3215935599407367,-2.0,1.9804729105323253,-0.2207011743610795,3.1349135470432454,2.5678744774817406],[0.01858342515292654,0.24686537447156315,0.3176342677145891,-0.14867004038313467,0.693939005036696,0.20668327086732097,0.1796162556288049,-0.6640189347913716,-0.2179697337554995,0.08315707661075396,-0.021582784799424437,0.008855358176534652,0.6132299308391888],0.001840081250422135),
([-0.7466812568441931,-2.0,1.996422538740978,0.08467891141937844,2.819346839824594,2.536022351760161],[0.047866879896408816,0.5900950484407771,0.6747246705812744,-0.21806722953575702,0.1697159294219099,0.5683495223478267,0.03761899042581645,-0.6346714860159639,-0.28894353392095856,0.11141549600073053,-0.10967297357486658,0.03755690376773783,0.6999924238403756],0.002034376687653787),
([-0.26158693917855813,-2.0,1.5771051603916053,0.14361178384020712,2.7805657274423816,2.2864215699677706],[1.1084768806286938,0.3037335789151708,0.7001326345170527,-0.16088592037691016,1.0836994234726354,0.110579483957211,0.3182561829683893,-0.7169917468407965,-0.3429736989579706,-0.5265413989667569,-0.6475034082111558,0.09315655748329693,0.46715723278251126],0.002853809953427602),
([-0.7157241929751302,-2.0,1.568949295073275,0.24626585994724493,2.9604835202497974,2.597066573880113],[0.3360993397958973,0.4477443856836842,0.8652743041394751,-0.4084870940191452,0.38878795203919403,0.07448295645618756,-0.24705055935245648,-0.6663243142988451,-0.085714782782274,-0.2982857149983497,-0.2572410875677202,0.10991489798314,0.5250756559822071],0.0030415114229961847),
([-0.7215947454092576,-2.0,1.6700263678434135,0.1817796632226386,3.249692661755636,2.2715730328859314],[0.11635474687202256,0.4678798854047774,0.7897018363667234,-0.3612407707559338,0.6420213238719674,0.2609977138010542,-0.26647166785211285,-0.7215278862160707,-0.31157306514471866,-0.27431023313888364,-0.4433373804544265,-0.01827828063245229,0.5154288469769379],0.003048502817898806)
]

    const gjs=[val[2]::Array{Float64,1} for val in best]
    const sjs=[val[1]::Array{Float64,1} for val in best]
    #const cjs=100.0*[val[3]::Float64 for val in best]
    const cjs=1.0 - exp(-(1/(2*1.0^2))*[val[3]::Float64 for val in best])

    for i=1:5
      add_sample!(crkr_inst, sjs[i], gjs[i], cjs[i])
    end


    #=
    const s0 = [-0.5,-1.382, 1.4090542+0.762007, 0., 3., 0.9812]::Array{Float64,1}
    const t_hit = 0.467361111111::Float64
    const g0 = [-0.67290765, 1.25480235, -1.07623605, -0.82715274, 1.57,0.80509373,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t_hit]::Array{Float64,1}
    const c0=10.0::Float64
    add_sample!(crkr_inst,s0,g0,c0)
    #add_sample!(mcrkr,s0,g0,c0) #mult
    =#
  end

  println(crkr_inst.σfs)
  println(crkr_inst.ls[1,:])
  #for n=1:13 println(crkr_inst.Γs[n]) end

  init_node("rosjl_example")
  pub_ms = Publisher("gazebo/set_model_state", ModelState, queue_size=50)  # used to publish visual markers in gazebo
  al_client = actionlib.SimpleActionClient("x6/arm_controller/follow_joint_trajectory", cmsg.FollowJointTrajectoryAction)

  robot, ikmodel = getOpenraveRobot()

  cjs, hts = [0.0], [0.0]#Array(Float64)
  t_client=actionlib.SimpleActionClient("find_result", optmsg.findResultAction)
  #t_client["wait_for_server"]()
  const R_to_ref = getRtoRef()::Array{Float64,2}

  #dmp_test_path = np.load('dmp_artificial_demonstration_6dof_x5.npy')
  #omp = OptUnconstrained(dmp_test_path, dt=0.001, n_bfs=60)


  #reused variables
  goal = optmsg.findResultGoal()
  s=TrainState(false,0,nTest,nLearn)::TrainState
  bs=ModelState()::optitrack_sim_tools.msg.ModelState
  zs6 = zeros(6) #auxiliar
  hit=true
  added=true
  #df = initSaveSamples()
  df = DataFrame(sj=Tuple{Array{Float64,1}}[], gj = Tuple{Array{Float64,1}}[], hit = Bool[], cj = Float64[], endEffectHitTime = Tuple{Array{Float64,1}}[], ballHitTime = Tuple{Array{Float64,1}}[], tablePos = Tuple{Array{Float64,1}}[])

  epoch=-1
  while epoch < n_epochs
    publish_trajectory.publish_home_trajectory(al_client) #send robot action
    rospy.rostime["wallsleep"](0.5) # Important simulates a spinOnce to allow the thread to continue

    if !s.isTraining || (hit && added) || updateAlways
      updateS!(s)::Tuple{Bool,Int64} #state machine
    end

    if s.isTraining && testOnly
      print_info(hit, s, crkr_inst, cj, cjs, hts)
      s.isTraining = false
      s.k = 0
      cjs[1]=0.0
      hts[1]=0.0
      continue
    end

    launchBs!(bs, s.isTraining, s.k, TEST_SET) #generate new play

    sj=[bs.pose.position.x, bs.pose.position.y, bs.pose.position.z, bs.twist.linear.x, bs.twist.linear.y, bs.twist.linear.z]
    gj=getGj(s.isTraining, crkr_inst, sj) #predict robot action
    #gj=getGj(s.isTraining, mcrkr, sj) #predict robot action
    joints, joints_v, thit = gj[1:6], gj[7:12], gj[13]

    robot["SetDOFValues"](gj[1:6], ikmodel["manip"]["GetArmIndices"]())
    if robot["CheckSelfCollision"]() println("Self Collision Detected") end #change to check entire trajectory on robot
    dir_kin_hit_time = R_to_ref*(ikmodel["manip"]["GetTransform"]()[1:4, 4])[1:4]

    #execute
    publish(pub_ms,bs) #send play to simulator
    time_hit=publish_trajectory.send_to_point_time_vel(al_client, joints, joints_v, thit) #send robot action

    #get feedback
    goal["time_hit"]=time_hit["to_sec"]()
    t_client["send_goal"](goal)
    t_client["wait_for_result"](rospy.Duration["from_sec"](3.0)) # wait for reply
    result=t_client["get_result"]()

    #interpret feedback
    if typeof(result) != Void
      res_bt=convert(Array{Float64,1},result["ball_table"])
      res_bthit=convert(Array{Float64,1},result["ball_thit"])
      (hit, res_bt) = check_success(res_bt)
      cj = cost_fun(TARGET, res_bt[1:3], res_bthit[1:3], dir_kin_hit_time[1:3])

      pub_markers(pub_ms, bs, res_bthit, dir_kin_hit_time, res_bt, zs6) # publish visual markers in gazebo

      #store sample
      #println([sj; gj; hit; cj; dir_kin_hit_time; res_bthit; res_bt])
      push!(df, [(sj,); (gj,); hit; cj; (dir_kin_hit_time,); (res_bthit,); (res_bt,)])
      if s.isTraining && (hit || updateAlways)
        added = add_sample!(crkr_inst,sj,gj,cj)::Bool
        #added = add_sample!(mcrkr,sj,gj,cj)::Bool
        added ? println("added sample"): println("rejected sample")
      end

      #log learning progress
      logProgress!(s, hit , cj, cjs, hts) # function to update logging
      if saveTraining && !s.isTraining && s.k==1 #save result
        save_training(crkr_inst, df)
        epoch += 1
      end

      #print stuff
      print_info(hit, s, crkr_inst, cj, cjs, hts)
      println("#################################")
      #println("sj: ",sj)
      #println("gj: ",gj)
      #print_info(hit, s, mcrkr.ks[1], cj, cjs, hts)
      #println("#################################")
   end

      #rospy.rostime["wallsleep"](0.5)
  end
  print_info(hit, crkr_inst, 0.0, cjs, hts, join([ARGS;".txt"])
)
  return crkr_inst
end

if length(ARGS) > 1
  ckr = run_node(parse(Float64,ARGS[1]), parse(Int64,ARGS[2]), parse(Int64,ARGS[3]))
else
  ckr = run_node()
end
