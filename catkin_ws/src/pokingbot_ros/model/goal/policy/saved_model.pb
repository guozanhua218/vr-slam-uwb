ç
Í£
8
Const
output"dtype"
valuetensor"
dtypetype

NoOp
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype
¾
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring 

VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 "serve*2.3.12v2.3.0-54-gfcc4b966f18Ô

policy_network/conv1_d/bVarHandleOp*
_output_shapes
: *
dtype0*
shape: *)
shared_namepolicy_network/conv1_d/b

,policy_network/conv1_d/b/Read/ReadVariableOpReadVariableOppolicy_network/conv1_d/b*
_output_shapes
: *
dtype0

policy_network/conv1_d/wVarHandleOp*
_output_shapes
: *
dtype0*
shape: *)
shared_namepolicy_network/conv1_d/w

,policy_network/conv1_d/w/Read/ReadVariableOpReadVariableOppolicy_network/conv1_d/w*"
_output_shapes
: *
dtype0

policy_network/conv1_d/b_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *+
shared_namepolicy_network/conv1_d/b_1

.policy_network/conv1_d/b_1/Read/ReadVariableOpReadVariableOppolicy_network/conv1_d/b_1*
_output_shapes
: *
dtype0

policy_network/conv1_d/w_1VarHandleOp*
_output_shapes
: *
dtype0*
shape:  *+
shared_namepolicy_network/conv1_d/w_1

.policy_network/conv1_d/w_1/Read/ReadVariableOpReadVariableOppolicy_network/conv1_d/w_1*"
_output_shapes
:  *
dtype0

policy_network/linear/bVarHandleOp*
_output_shapes
: *
dtype0*
shape:*(
shared_namepolicy_network/linear/b

+policy_network/linear/b/Read/ReadVariableOpReadVariableOppolicy_network/linear/b*
_output_shapes	
:*
dtype0

policy_network/linear/wVarHandleOp*
_output_shapes
: *
dtype0*
shape:
¾<*(
shared_namepolicy_network/linear/w

+policy_network/linear/w/Read/ReadVariableOpReadVariableOppolicy_network/linear/w* 
_output_shapes
:
¾<*
dtype0

policy_network/linear/b_1VarHandleOp*
_output_shapes
: *
dtype0*
shape:**
shared_namepolicy_network/linear/b_1

-policy_network/linear/b_1/Read/ReadVariableOpReadVariableOppolicy_network/linear/b_1*
_output_shapes	
:*
dtype0

policy_network/linear/w_1VarHandleOp*
_output_shapes
: *
dtype0*
shape:
**
shared_namepolicy_network/linear/w_1

-policy_network/linear/w_1/Read/ReadVariableOpReadVariableOppolicy_network/linear/w_1* 
_output_shapes
:
*
dtype0

policy_network/linear/b_2VarHandleOp*
_output_shapes
: *
dtype0*
shape:**
shared_namepolicy_network/linear/b_2

-policy_network/linear/b_2/Read/ReadVariableOpReadVariableOppolicy_network/linear/b_2*
_output_shapes	
:*
dtype0

policy_network/linear/w_2VarHandleOp*
_output_shapes
: *
dtype0*
shape:
**
shared_namepolicy_network/linear/w_2

-policy_network/linear/w_2/Read/ReadVariableOpReadVariableOppolicy_network/linear/w_2* 
_output_shapes
:
*
dtype0

policy_network/linear/b_3VarHandleOp*
_output_shapes
: *
dtype0*
shape:**
shared_namepolicy_network/linear/b_3

-policy_network/linear/b_3/Read/ReadVariableOpReadVariableOppolicy_network/linear/b_3*
_output_shapes	
:*
dtype0

policy_network/linear/w_3VarHandleOp*
_output_shapes
: *
dtype0*
shape:
**
shared_namepolicy_network/linear/w_3

-policy_network/linear/w_3/Read/ReadVariableOpReadVariableOppolicy_network/linear/w_3* 
_output_shapes
:
*
dtype0

policy_network/linear/b_4VarHandleOp*
_output_shapes
: *
dtype0*
shape:**
shared_namepolicy_network/linear/b_4

-policy_network/linear/b_4/Read/ReadVariableOpReadVariableOppolicy_network/linear/b_4*
_output_shapes
:*
dtype0

policy_network/linear/w_4VarHandleOp*
_output_shapes
: *
dtype0*
shape:	**
shared_namepolicy_network/linear/w_4

-policy_network/linear/w_4/Read/ReadVariableOpReadVariableOppolicy_network/linear/w_4*
_output_shapes
:	*
dtype0

NoOpNoOp
·
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*ò
valueèBå BÞ
:

_variables
_trainable_variables

signatures
f
0
1
2
3
4
	5

6
7
8
9
10
11
12
13
f
0
1
2
3
4
	5

6
7
8
9
10
11
12
13
 
US
VARIABLE_VALUEpolicy_network/conv1_d/b'_variables/0/.ATTRIBUTES/VARIABLE_VALUE
US
VARIABLE_VALUEpolicy_network/conv1_d/w'_variables/1/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEpolicy_network/conv1_d/b_1'_variables/2/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEpolicy_network/conv1_d/w_1'_variables/3/.ATTRIBUTES/VARIABLE_VALUE
TR
VARIABLE_VALUEpolicy_network/linear/b'_variables/4/.ATTRIBUTES/VARIABLE_VALUE
TR
VARIABLE_VALUEpolicy_network/linear/w'_variables/5/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEpolicy_network/linear/b_1'_variables/6/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEpolicy_network/linear/w_1'_variables/7/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEpolicy_network/linear/b_2'_variables/8/.ATTRIBUTES/VARIABLE_VALUE
VT
VARIABLE_VALUEpolicy_network/linear/w_2'_variables/9/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEpolicy_network/linear/b_3(_variables/10/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEpolicy_network/linear/w_3(_variables/11/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEpolicy_network/linear/b_4(_variables/12/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEpolicy_network/linear/w_4(_variables/13/.ATTRIBUTES/VARIABLE_VALUE
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
»
StatefulPartitionedCallStatefulPartitionedCallsaver_filename,policy_network/conv1_d/b/Read/ReadVariableOp,policy_network/conv1_d/w/Read/ReadVariableOp.policy_network/conv1_d/b_1/Read/ReadVariableOp.policy_network/conv1_d/w_1/Read/ReadVariableOp+policy_network/linear/b/Read/ReadVariableOp+policy_network/linear/w/Read/ReadVariableOp-policy_network/linear/b_1/Read/ReadVariableOp-policy_network/linear/w_1/Read/ReadVariableOp-policy_network/linear/b_2/Read/ReadVariableOp-policy_network/linear/w_2/Read/ReadVariableOp-policy_network/linear/b_3/Read/ReadVariableOp-policy_network/linear/w_3/Read/ReadVariableOp-policy_network/linear/b_4/Read/ReadVariableOp-policy_network/linear/w_4/Read/ReadVariableOpConst*
Tin
2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *2
config_proto" 

CPU

GPU2 *0J 8 *)
f$R"
 __inference__traced_save_7571062
 
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filenamepolicy_network/conv1_d/bpolicy_network/conv1_d/wpolicy_network/conv1_d/b_1policy_network/conv1_d/w_1policy_network/linear/bpolicy_network/linear/wpolicy_network/linear/b_1policy_network/linear/w_1policy_network/linear/b_2policy_network/linear/w_2policy_network/linear/b_3policy_network/linear/w_3policy_network/linear/b_4policy_network/linear/w_4*
Tin
2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *2
config_proto" 

CPU

GPU2 *0J 8 *,
f'R%
#__inference__traced_restore_7571114µ
)

 __inference__traced_save_7571062
file_prefix7
3savev2_policy_network_conv1_d_b_read_readvariableop7
3savev2_policy_network_conv1_d_w_read_readvariableop9
5savev2_policy_network_conv1_d_b_1_read_readvariableop9
5savev2_policy_network_conv1_d_w_1_read_readvariableop6
2savev2_policy_network_linear_b_read_readvariableop6
2savev2_policy_network_linear_w_read_readvariableop8
4savev2_policy_network_linear_b_1_read_readvariableop8
4savev2_policy_network_linear_w_1_read_readvariableop8
4savev2_policy_network_linear_b_2_read_readvariableop8
4savev2_policy_network_linear_w_2_read_readvariableop8
4savev2_policy_network_linear_b_3_read_readvariableop8
4savev2_policy_network_linear_w_3_read_readvariableop8
4savev2_policy_network_linear_b_4_read_readvariableop8
4savev2_policy_network_linear_w_4_read_readvariableop
savev2_const

identity_1¢MergeV2Checkpoints
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*2
StaticRegexFullMatchc
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.part2
Const
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*<
value3B1 B+_temp_f43b61f084264a0d977b61087e29f790/part2	
Const_1
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: 2
Selectt

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: 2

StringJoinZ

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :2

num_shards
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 2
ShardedFilename/shard¦
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: 2
ShardedFilenameã
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*õ
valueëBèB'_variables/0/.ATTRIBUTES/VARIABLE_VALUEB'_variables/1/.ATTRIBUTES/VARIABLE_VALUEB'_variables/2/.ATTRIBUTES/VARIABLE_VALUEB'_variables/3/.ATTRIBUTES/VARIABLE_VALUEB'_variables/4/.ATTRIBUTES/VARIABLE_VALUEB'_variables/5/.ATTRIBUTES/VARIABLE_VALUEB'_variables/6/.ATTRIBUTES/VARIABLE_VALUEB'_variables/7/.ATTRIBUTES/VARIABLE_VALUEB'_variables/8/.ATTRIBUTES/VARIABLE_VALUEB'_variables/9/.ATTRIBUTES/VARIABLE_VALUEB(_variables/10/.ATTRIBUTES/VARIABLE_VALUEB(_variables/11/.ATTRIBUTES/VARIABLE_VALUEB(_variables/12/.ATTRIBUTES/VARIABLE_VALUEB(_variables/13/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
SaveV2/tensor_names¦
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*1
value(B&B B B B B B B B B B B B B B B 2
SaveV2/shape_and_slices¸
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:03savev2_policy_network_conv1_d_b_read_readvariableop3savev2_policy_network_conv1_d_w_read_readvariableop5savev2_policy_network_conv1_d_b_1_read_readvariableop5savev2_policy_network_conv1_d_w_1_read_readvariableop2savev2_policy_network_linear_b_read_readvariableop2savev2_policy_network_linear_w_read_readvariableop4savev2_policy_network_linear_b_1_read_readvariableop4savev2_policy_network_linear_w_1_read_readvariableop4savev2_policy_network_linear_b_2_read_readvariableop4savev2_policy_network_linear_w_2_read_readvariableop4savev2_policy_network_linear_b_3_read_readvariableop4savev2_policy_network_linear_w_3_read_readvariableop4savev2_policy_network_linear_b_4_read_readvariableop4savev2_policy_network_linear_w_4_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 *
dtypes
22
SaveV2º
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:2(
&MergeV2Checkpoints/checkpoint_prefixes¡
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 2
MergeV2Checkpointsr
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: 2

Identitym

Identity_1IdentityIdentity:output:0^MergeV2Checkpoints*
T0*
_output_shapes
: 2

Identity_1"!

identity_1Identity_1:output:0*
_input_shapes
: : : : :  ::
¾<::
::
::
::	: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix: 

_output_shapes
: :($
"
_output_shapes
: : 

_output_shapes
: :($
"
_output_shapes
:  :!

_output_shapes	
::&"
 
_output_shapes
:
¾<:!

_output_shapes	
::&"
 
_output_shapes
:
:!	

_output_shapes	
::&
"
 
_output_shapes
:
:!

_output_shapes	
::&"
 
_output_shapes
:
: 

_output_shapes
::%!

_output_shapes
:	:

_output_shapes
: 
¶	
¦
__inference___call___27733

args_0
unknown
	unknown_0
	unknown_1
	unknown_2
	unknown_3
	unknown_4
	unknown_5
	unknown_6
	unknown_7
	unknown_8
	unknown_9

unknown_10

unknown_11

unknown_12
identity¢StatefulPartitionedCalló
StatefulPartitionedCallStatefulPartitionedCallargs_0unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ*0
_read_only_resource_inputs
	
*2
config_proto" 

CPU

GPU2 *0J 8 *(
f#R!
__inference_wrapped_module_10612
StatefulPartitionedCall
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2

Identity"
identityIdentity:output:0*_
_input_shapesN
L:ÿÿÿÿÿÿÿÿÿâ::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:ÿÿÿÿÿÿÿÿÿâ
 
_user_specified_nameargs_0
®

 __inference_wrapped_module_27839

args_0K
Gpolicy_network_conv1_d_convolution_expanddims_1_readvariableop_resource:
6policy_network_conv1_d_biasadd_readvariableop_resourceM
Ipolicy_network_conv1_d_convolution_1_expanddims_1_readvariableop_resource<
8policy_network_conv1_d_biasadd_1_readvariableop_resource8
4policy_network_linear_matmul_readvariableop_resource5
1policy_network_linear_add_readvariableop_resource:
6policy_network_linear_matmul_1_readvariableop_resource7
3policy_network_linear_add_1_readvariableop_resource:
6policy_network_linear_matmul_2_readvariableop_resource7
3policy_network_linear_add_2_readvariableop_resource:
6policy_network_linear_matmul_3_readvariableop_resource7
3policy_network_linear_add_3_readvariableop_resource:
6policy_network_linear_matmul_4_readvariableop_resource7
3policy_network_linear_add_4_readvariableop_resource
identityr
policy_network/flatten/ShapeShapeargs_0*
T0*
_output_shapes
:2
policy_network/flatten/Shape¢
*policy_network/flatten/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2,
*policy_network/flatten/strided_slice/stack¦
,policy_network/flatten/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/flatten/strided_slice/stack_1¦
,policy_network/flatten/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/flatten/strided_slice/stack_2ê
$policy_network/flatten/strided_sliceStridedSlice%policy_network/flatten/Shape:output:03policy_network/flatten/strided_slice/stack:output:05policy_network/flatten/strided_slice/stack_1:output:05policy_network/flatten/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2&
$policy_network/flatten/strided_slice
&policy_network/flatten/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB:â2(
&policy_network/flatten/concat/values_1
"policy_network/flatten/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2$
"policy_network/flatten/concat/axis
policy_network/flatten/concatConcatV2-policy_network/flatten/strided_slice:output:0/policy_network/flatten/concat/values_1:output:0+policy_network/flatten/concat/axis:output:0*
N*
T0*
_output_shapes
:2
policy_network/flatten/concat®
policy_network/flatten/ReshapeReshapeargs_0&policy_network/flatten/concat:output:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿâ2 
policy_network/flatten/Reshape
 policy_network/concat/concat_dimConst*
_output_shapes
: *
dtype0*
valueB :
ÿÿÿÿÿÿÿÿÿ2"
 policy_network/concat/concat_dim¤
policy_network/concat/concatIdentity'policy_network/flatten/Reshape:output:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿâ2
policy_network/concat/concat}
policy_network/ConstConst*
_output_shapes
:*
dtype0*
valueB"Ä     2
policy_network/Const
policy_network/split/split_dimConst*
_output_shapes
: *
dtype0*
value	B :2 
policy_network/split/split_dim
policy_network/splitSplitV%policy_network/concat/concat:output:0policy_network/Const:output:0'policy_network/split/split_dim:output:0*
T0*

Tlen0*;
_output_shapes)
':ÿÿÿÿÿÿÿÿÿÄ:ÿÿÿÿÿÿÿÿÿ*
	num_split2
policy_network/split
policy_network/reshape/ShapeShapepolicy_network/split:output:0*
T0*
_output_shapes
:2
policy_network/reshape/Shape¢
*policy_network/reshape/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2,
*policy_network/reshape/strided_slice/stack¦
,policy_network/reshape/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/reshape/strided_slice/stack_1¦
,policy_network/reshape/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/reshape/strided_slice/stack_2ê
$policy_network/reshape/strided_sliceStridedSlice%policy_network/reshape/Shape:output:03policy_network/reshape/strided_slice/stack:output:05policy_network/reshape/strided_slice/stack_1:output:05policy_network/reshape/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2&
$policy_network/reshape/strided_slice¡
&policy_network/reshape/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB"ñ      2(
&policy_network/reshape/concat/values_1
"policy_network/reshape/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2$
"policy_network/reshape/concat/axis
policy_network/reshape/concatConcatV2-policy_network/reshape/strided_slice:output:0/policy_network/reshape/concat/values_1:output:0+policy_network/reshape/concat/axis:output:0*
N*
T0*
_output_shapes
:2
policy_network/reshape/concatÉ
policy_network/reshape/ReshapeReshapepolicy_network/split:output:0&policy_network/reshape/concat:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ2 
policy_network/reshape/Reshape±
1policy_network/conv1_d/convolution/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ýÿÿÿÿÿÿÿÿ23
1policy_network/conv1_d/convolution/ExpandDims/dim
-policy_network/conv1_d/convolution/ExpandDims
ExpandDims'policy_network/reshape/Reshape:output:0:policy_network/conv1_d/convolution/ExpandDims/dim:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ2/
-policy_network/conv1_d/convolution/ExpandDims
>policy_network/conv1_d/convolution/ExpandDims_1/ReadVariableOpReadVariableOpGpolicy_network_conv1_d_convolution_expanddims_1_readvariableop_resource*"
_output_shapes
: *
dtype02@
>policy_network/conv1_d/convolution/ExpandDims_1/ReadVariableOp¬
3policy_network/conv1_d/convolution/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 25
3policy_network/conv1_d/convolution/ExpandDims_1/dim§
/policy_network/conv1_d/convolution/ExpandDims_1
ExpandDimsFpolicy_network/conv1_d/convolution/ExpandDims_1/ReadVariableOp:value:0<policy_network/conv1_d/convolution/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
: 21
/policy_network/conv1_d/convolution/ExpandDims_1§
"policy_network/conv1_d/convolutionConv2D6policy_network/conv1_d/convolution/ExpandDims:output:08policy_network/conv1_d/convolution/ExpandDims_1:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
paddingSAME*
strides
2$
"policy_network/conv1_d/convolutionç
*policy_network/conv1_d/convolution/SqueezeSqueeze+policy_network/conv1_d/convolution:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
squeeze_dims

ýÿÿÿÿÿÿÿÿ2,
*policy_network/conv1_d/convolution/SqueezeÑ
-policy_network/conv1_d/BiasAdd/ReadVariableOpReadVariableOp6policy_network_conv1_d_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02/
-policy_network/conv1_d/BiasAdd/ReadVariableOpî
policy_network/conv1_d/BiasAddBiasAdd3policy_network/conv1_d/convolution/Squeeze:output:05policy_network/conv1_d/BiasAdd/ReadVariableOp:value:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2 
policy_network/conv1_d/BiasAdd¨
policy_network/sequential/ReluRelu'policy_network/conv1_d/BiasAdd:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2 
policy_network/sequential/Reluµ
3policy_network/conv1_d/convolution_1/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ýÿÿÿÿÿÿÿÿ25
3policy_network/conv1_d/convolution_1/ExpandDims/dim
/policy_network/conv1_d/convolution_1/ExpandDims
ExpandDims,policy_network/sequential/Relu:activations:0<policy_network/conv1_d/convolution_1/ExpandDims/dim:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 21
/policy_network/conv1_d/convolution_1/ExpandDims
@policy_network/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpReadVariableOpIpolicy_network_conv1_d_convolution_1_expanddims_1_readvariableop_resource*"
_output_shapes
:  *
dtype02B
@policy_network/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp°
5policy_network/conv1_d/convolution_1/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 27
5policy_network/conv1_d/convolution_1/ExpandDims_1/dim¯
1policy_network/conv1_d/convolution_1/ExpandDims_1
ExpandDimsHpolicy_network/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp:value:0>policy_network/conv1_d/convolution_1/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:  23
1policy_network/conv1_d/convolution_1/ExpandDims_1¯
$policy_network/conv1_d/convolution_1Conv2D8policy_network/conv1_d/convolution_1/ExpandDims:output:0:policy_network/conv1_d/convolution_1/ExpandDims_1:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
paddingSAME*
strides
2&
$policy_network/conv1_d/convolution_1í
,policy_network/conv1_d/convolution_1/SqueezeSqueeze-policy_network/conv1_d/convolution_1:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
squeeze_dims

ýÿÿÿÿÿÿÿÿ2.
,policy_network/conv1_d/convolution_1/Squeeze×
/policy_network/conv1_d/BiasAdd_1/ReadVariableOpReadVariableOp8policy_network_conv1_d_biasadd_1_readvariableop_resource*
_output_shapes
: *
dtype021
/policy_network/conv1_d/BiasAdd_1/ReadVariableOpö
 policy_network/conv1_d/BiasAdd_1BiasAdd5policy_network/conv1_d/convolution_1/Squeeze:output:07policy_network/conv1_d/BiasAdd_1/ReadVariableOp:value:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2"
 policy_network/conv1_d/BiasAdd_1®
 policy_network/sequential/Relu_1Relu)policy_network/conv1_d/BiasAdd_1:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2"
 policy_network/sequential/Relu_1
policy_network/flatten/Shape_1Shape.policy_network/sequential/Relu_1:activations:0*
T0*
_output_shapes
:2 
policy_network/flatten/Shape_1¦
,policy_network/flatten/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB: 2.
,policy_network/flatten/strided_slice_1/stackª
.policy_network/flatten/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.policy_network/flatten/strided_slice_1/stack_1ª
.policy_network/flatten/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.policy_network/flatten/strided_slice_1/stack_2ö
&policy_network/flatten/strided_slice_1StridedSlice'policy_network/flatten/Shape_1:output:05policy_network/flatten/strided_slice_1/stack:output:07policy_network/flatten/strided_slice_1/stack_1:output:07policy_network/flatten/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2(
&policy_network/flatten/strided_slice_1
(policy_network/flatten/concat_1/values_1Const*
_output_shapes
:*
dtype0*
valueB: <2*
(policy_network/flatten/concat_1/values_1
$policy_network/flatten/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B : 2&
$policy_network/flatten/concat_1/axis
policy_network/flatten/concat_1ConcatV2/policy_network/flatten/strided_slice_1:output:01policy_network/flatten/concat_1/values_1:output:0-policy_network/flatten/concat_1/axis:output:0*
N*
T0*
_output_shapes
:2!
policy_network/flatten/concat_1Ü
 policy_network/flatten/Reshape_1Reshape.policy_network/sequential/Relu_1:activations:0(policy_network/flatten/concat_1:output:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ <2"
 policy_network/flatten/Reshape_1~
policy_network/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B :2
policy_network/concat_1/axisë
policy_network/concat_1ConcatV2policy_network/split:output:1)policy_network/flatten/Reshape_1:output:0%policy_network/concat_1/axis:output:0*
N*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ¾<2
policy_network/concat_1Ñ
+policy_network/linear/MatMul/ReadVariableOpReadVariableOp4policy_network_linear_matmul_readvariableop_resource* 
_output_shapes
:
¾<*
dtype02-
+policy_network/linear/MatMul/ReadVariableOpÐ
policy_network/linear/MatMulMatMul policy_network/concat_1:output:03policy_network/linear/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/MatMulÃ
(policy_network/linear/Add/ReadVariableOpReadVariableOp1policy_network_linear_add_readvariableop_resource*
_output_shapes	
:*
dtype02*
(policy_network/linear/Add/ReadVariableOpÊ
policy_network/linear/AddAdd&policy_network/linear/MatMul:product:00policy_network/linear/Add/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add
policy_network/sequential/EluElupolicy_network/linear/Add:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/sequential/Elu×
-policy_network/linear/MatMul_1/ReadVariableOpReadVariableOp6policy_network_linear_matmul_1_readvariableop_resource* 
_output_shapes
:
*
dtype02/
-policy_network/linear/MatMul_1/ReadVariableOpá
policy_network/linear/MatMul_1MatMul+policy_network/sequential/Elu:activations:05policy_network/linear/MatMul_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_1É
*policy_network/linear/Add_1/ReadVariableOpReadVariableOp3policy_network_linear_add_1_readvariableop_resource*
_output_shapes	
:*
dtype02,
*policy_network/linear/Add_1/ReadVariableOpÒ
policy_network/linear/Add_1Add(policy_network/linear/MatMul_1:product:02policy_network/linear/Add_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_1
policy_network/sequential/Elu_1Elupolicy_network/linear/Add_1:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/sequential/Elu_1×
-policy_network/linear/MatMul_2/ReadVariableOpReadVariableOp6policy_network_linear_matmul_2_readvariableop_resource* 
_output_shapes
:
*
dtype02/
-policy_network/linear/MatMul_2/ReadVariableOpã
policy_network/linear/MatMul_2MatMul-policy_network/sequential/Elu_1:activations:05policy_network/linear/MatMul_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_2É
*policy_network/linear/Add_2/ReadVariableOpReadVariableOp3policy_network_linear_add_2_readvariableop_resource*
_output_shapes	
:*
dtype02,
*policy_network/linear/Add_2/ReadVariableOpÒ
policy_network/linear/Add_2Add(policy_network/linear/MatMul_2:product:02policy_network/linear/Add_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_2
policy_network/sequential/Elu_2Elupolicy_network/linear/Add_2:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/sequential/Elu_2×
-policy_network/linear/MatMul_3/ReadVariableOpReadVariableOp6policy_network_linear_matmul_3_readvariableop_resource* 
_output_shapes
:
*
dtype02/
-policy_network/linear/MatMul_3/ReadVariableOpã
policy_network/linear/MatMul_3MatMul-policy_network/sequential/Elu_2:activations:05policy_network/linear/MatMul_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_3É
*policy_network/linear/Add_3/ReadVariableOpReadVariableOp3policy_network_linear_add_3_readvariableop_resource*
_output_shapes	
:*
dtype02,
*policy_network/linear/Add_3/ReadVariableOpÒ
policy_network/linear/Add_3Add(policy_network/linear/MatMul_3:product:02policy_network/linear/Add_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_3
policy_network/sequential/Elu_3Elupolicy_network/linear/Add_3:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/sequential/Elu_3Ö
-policy_network/linear/MatMul_4/ReadVariableOpReadVariableOp6policy_network_linear_matmul_4_readvariableop_resource*
_output_shapes
:	*
dtype02/
-policy_network/linear/MatMul_4/ReadVariableOpâ
policy_network/linear/MatMul_4MatMul-policy_network/sequential/Elu_3:activations:05policy_network/linear/MatMul_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_4È
*policy_network/linear/Add_4/ReadVariableOpReadVariableOp3policy_network_linear_add_4_readvariableop_resource*
_output_shapes
:*
dtype02,
*policy_network/linear/Add_4/ReadVariableOpÑ
policy_network/linear/Add_4Add(policy_network/linear/MatMul_4:product:02policy_network/linear/Add_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_4
 policy_network/tanh_to_spec/TanhTanhpolicy_network/linear/Add_4:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2"
 policy_network/tanh_to_spec/Tanh
!policy_network/tanh_to_spec/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *  ?2#
!policy_network/tanh_to_spec/add/yÏ
policy_network/tanh_to_spec/addAddV2$policy_network/tanh_to_spec/Tanh:y:0*policy_network/tanh_to_spec/add/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/tanh_to_spec/add
!policy_network/tanh_to_spec/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2#
!policy_network/tanh_to_spec/mul/xÌ
policy_network/tanh_to_spec/mulMul*policy_network/tanh_to_spec/mul/x:output:0#policy_network/tanh_to_spec/add:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/tanh_to_spec/mul
#policy_network/tanh_to_spec/mul_1/yConst*
_output_shapes
:*
dtype0*
valueB"  ?   @2%
#policy_network/tanh_to_spec/mul_1/yÒ
!policy_network/tanh_to_spec/mul_1Mul#policy_network/tanh_to_spec/mul:z:0,policy_network/tanh_to_spec/mul_1/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2#
!policy_network/tanh_to_spec/mul_1
#policy_network/tanh_to_spec/add_1/yConst*
_output_shapes
:*
dtype0*
valueB"      ¿2%
#policy_network/tanh_to_spec/add_1/yÖ
!policy_network/tanh_to_spec/add_1AddV2%policy_network/tanh_to_spec/mul_1:z:0,policy_network/tanh_to_spec/add_1/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2#
!policy_network/tanh_to_spec/add_1y
IdentityIdentity%policy_network/tanh_to_spec/add_1:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2

Identity"
identityIdentity:output:0*_
_input_shapesN
L:ÿÿÿÿÿÿÿÿÿâ:::::::::::::::P L
(
_output_shapes
:ÿÿÿÿÿÿÿÿÿâ
 
_user_specified_nameargs_0
þ=
¬
#__inference__traced_restore_7571114
file_prefix-
)assignvariableop_policy_network_conv1_d_b/
+assignvariableop_1_policy_network_conv1_d_w1
-assignvariableop_2_policy_network_conv1_d_b_11
-assignvariableop_3_policy_network_conv1_d_w_1.
*assignvariableop_4_policy_network_linear_b.
*assignvariableop_5_policy_network_linear_w0
,assignvariableop_6_policy_network_linear_b_10
,assignvariableop_7_policy_network_linear_w_10
,assignvariableop_8_policy_network_linear_b_20
,assignvariableop_9_policy_network_linear_w_21
-assignvariableop_10_policy_network_linear_b_31
-assignvariableop_11_policy_network_linear_w_31
-assignvariableop_12_policy_network_linear_b_41
-assignvariableop_13_policy_network_linear_w_4
identity_15¢AssignVariableOp¢AssignVariableOp_1¢AssignVariableOp_10¢AssignVariableOp_11¢AssignVariableOp_12¢AssignVariableOp_13¢AssignVariableOp_2¢AssignVariableOp_3¢AssignVariableOp_4¢AssignVariableOp_5¢AssignVariableOp_6¢AssignVariableOp_7¢AssignVariableOp_8¢AssignVariableOp_9é
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*õ
valueëBèB'_variables/0/.ATTRIBUTES/VARIABLE_VALUEB'_variables/1/.ATTRIBUTES/VARIABLE_VALUEB'_variables/2/.ATTRIBUTES/VARIABLE_VALUEB'_variables/3/.ATTRIBUTES/VARIABLE_VALUEB'_variables/4/.ATTRIBUTES/VARIABLE_VALUEB'_variables/5/.ATTRIBUTES/VARIABLE_VALUEB'_variables/6/.ATTRIBUTES/VARIABLE_VALUEB'_variables/7/.ATTRIBUTES/VARIABLE_VALUEB'_variables/8/.ATTRIBUTES/VARIABLE_VALUEB'_variables/9/.ATTRIBUTES/VARIABLE_VALUEB(_variables/10/.ATTRIBUTES/VARIABLE_VALUEB(_variables/11/.ATTRIBUTES/VARIABLE_VALUEB(_variables/12/.ATTRIBUTES/VARIABLE_VALUEB(_variables/13/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
RestoreV2/tensor_names¬
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*1
value(B&B B B B B B B B B B B B B B B 2
RestoreV2/shape_and_slicesö
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*P
_output_shapes>
<:::::::::::::::*
dtypes
22
	RestoreV2g
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:2

Identity¨
AssignVariableOpAssignVariableOp)assignvariableop_policy_network_conv1_d_bIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1°
AssignVariableOp_1AssignVariableOp+assignvariableop_1_policy_network_conv1_d_wIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2²
AssignVariableOp_2AssignVariableOp-assignvariableop_2_policy_network_conv1_d_b_1Identity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3²
AssignVariableOp_3AssignVariableOp-assignvariableop_3_policy_network_conv1_d_w_1Identity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4¯
AssignVariableOp_4AssignVariableOp*assignvariableop_4_policy_network_linear_bIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5¯
AssignVariableOp_5AssignVariableOp*assignvariableop_5_policy_network_linear_wIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_5k

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:2

Identity_6±
AssignVariableOp_6AssignVariableOp,assignvariableop_6_policy_network_linear_b_1Identity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_6k

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:2

Identity_7±
AssignVariableOp_7AssignVariableOp,assignvariableop_7_policy_network_linear_w_1Identity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_7k

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:2

Identity_8±
AssignVariableOp_8AssignVariableOp,assignvariableop_8_policy_network_linear_b_2Identity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_8k

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:2

Identity_9±
AssignVariableOp_9AssignVariableOp,assignvariableop_9_policy_network_linear_w_2Identity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_9n
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:2
Identity_10µ
AssignVariableOp_10AssignVariableOp-assignvariableop_10_policy_network_linear_b_3Identity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_10n
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:2
Identity_11µ
AssignVariableOp_11AssignVariableOp-assignvariableop_11_policy_network_linear_w_3Identity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_11n
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:2
Identity_12µ
AssignVariableOp_12AssignVariableOp-assignvariableop_12_policy_network_linear_b_4Identity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_12n
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:2
Identity_13µ
AssignVariableOp_13AssignVariableOp-assignvariableop_13_policy_network_linear_w_4Identity_13:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_139
NoOpNoOp"/device:CPU:0*
_output_shapes
 2
NoOp
Identity_14Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9^NoOp"/device:CPU:0*
T0*
_output_shapes
: 2
Identity_14
Identity_15IdentityIdentity_14:output:0^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_2^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9*
T0*
_output_shapes
: 2
Identity_15"#
identity_15Identity_15:output:0*M
_input_shapes<
:: ::::::::::::::2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12*
AssignVariableOp_10AssignVariableOp_102*
AssignVariableOp_11AssignVariableOp_112*
AssignVariableOp_12AssignVariableOp_122*
AssignVariableOp_13AssignVariableOp_132(
AssignVariableOp_2AssignVariableOp_22(
AssignVariableOp_3AssignVariableOp_32(
AssignVariableOp_4AssignVariableOp_42(
AssignVariableOp_5AssignVariableOp_52(
AssignVariableOp_6AssignVariableOp_62(
AssignVariableOp_7AssignVariableOp_72(
AssignVariableOp_8AssignVariableOp_82(
AssignVariableOp_9AssignVariableOp_9:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
­

__inference_wrapped_module_1061

args_0K
Gpolicy_network_conv1_d_convolution_expanddims_1_readvariableop_resource:
6policy_network_conv1_d_biasadd_readvariableop_resourceM
Ipolicy_network_conv1_d_convolution_1_expanddims_1_readvariableop_resource<
8policy_network_conv1_d_biasadd_1_readvariableop_resource8
4policy_network_linear_matmul_readvariableop_resource5
1policy_network_linear_add_readvariableop_resource:
6policy_network_linear_matmul_1_readvariableop_resource7
3policy_network_linear_add_1_readvariableop_resource:
6policy_network_linear_matmul_2_readvariableop_resource7
3policy_network_linear_add_2_readvariableop_resource:
6policy_network_linear_matmul_3_readvariableop_resource7
3policy_network_linear_add_3_readvariableop_resource:
6policy_network_linear_matmul_4_readvariableop_resource7
3policy_network_linear_add_4_readvariableop_resource
identityr
policy_network/flatten/ShapeShapeargs_0*
T0*
_output_shapes
:2
policy_network/flatten/Shape¢
*policy_network/flatten/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2,
*policy_network/flatten/strided_slice/stack¦
,policy_network/flatten/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/flatten/strided_slice/stack_1¦
,policy_network/flatten/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/flatten/strided_slice/stack_2ê
$policy_network/flatten/strided_sliceStridedSlice%policy_network/flatten/Shape:output:03policy_network/flatten/strided_slice/stack:output:05policy_network/flatten/strided_slice/stack_1:output:05policy_network/flatten/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2&
$policy_network/flatten/strided_slice
&policy_network/flatten/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB:â2(
&policy_network/flatten/concat/values_1
"policy_network/flatten/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2$
"policy_network/flatten/concat/axis
policy_network/flatten/concatConcatV2-policy_network/flatten/strided_slice:output:0/policy_network/flatten/concat/values_1:output:0+policy_network/flatten/concat/axis:output:0*
N*
T0*
_output_shapes
:2
policy_network/flatten/concat®
policy_network/flatten/ReshapeReshapeargs_0&policy_network/flatten/concat:output:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿâ2 
policy_network/flatten/Reshape
 policy_network/concat/concat_dimConst*
_output_shapes
: *
dtype0*
valueB :
ÿÿÿÿÿÿÿÿÿ2"
 policy_network/concat/concat_dim¤
policy_network/concat/concatIdentity'policy_network/flatten/Reshape:output:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿâ2
policy_network/concat/concat}
policy_network/ConstConst*
_output_shapes
:*
dtype0*
valueB"Ä     2
policy_network/Const
policy_network/split/split_dimConst*
_output_shapes
: *
dtype0*
value	B :2 
policy_network/split/split_dim
policy_network/splitSplitV%policy_network/concat/concat:output:0policy_network/Const:output:0'policy_network/split/split_dim:output:0*
T0*

Tlen0*;
_output_shapes)
':ÿÿÿÿÿÿÿÿÿÄ:ÿÿÿÿÿÿÿÿÿ*
	num_split2
policy_network/split
policy_network/reshape/ShapeShapepolicy_network/split:output:0*
T0*
_output_shapes
:2
policy_network/reshape/Shape¢
*policy_network/reshape/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 2,
*policy_network/reshape/strided_slice/stack¦
,policy_network/reshape/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/reshape/strided_slice/stack_1¦
,policy_network/reshape/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:2.
,policy_network/reshape/strided_slice/stack_2ê
$policy_network/reshape/strided_sliceStridedSlice%policy_network/reshape/Shape:output:03policy_network/reshape/strided_slice/stack:output:05policy_network/reshape/strided_slice/stack_1:output:05policy_network/reshape/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2&
$policy_network/reshape/strided_slice¡
&policy_network/reshape/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB"ñ      2(
&policy_network/reshape/concat/values_1
"policy_network/reshape/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2$
"policy_network/reshape/concat/axis
policy_network/reshape/concatConcatV2-policy_network/reshape/strided_slice:output:0/policy_network/reshape/concat/values_1:output:0+policy_network/reshape/concat/axis:output:0*
N*
T0*
_output_shapes
:2
policy_network/reshape/concatÉ
policy_network/reshape/ReshapeReshapepolicy_network/split:output:0&policy_network/reshape/concat:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ2 
policy_network/reshape/Reshape±
1policy_network/conv1_d/convolution/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ýÿÿÿÿÿÿÿÿ23
1policy_network/conv1_d/convolution/ExpandDims/dim
-policy_network/conv1_d/convolution/ExpandDims
ExpandDims'policy_network/reshape/Reshape:output:0:policy_network/conv1_d/convolution/ExpandDims/dim:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ2/
-policy_network/conv1_d/convolution/ExpandDims
>policy_network/conv1_d/convolution/ExpandDims_1/ReadVariableOpReadVariableOpGpolicy_network_conv1_d_convolution_expanddims_1_readvariableop_resource*"
_output_shapes
: *
dtype02@
>policy_network/conv1_d/convolution/ExpandDims_1/ReadVariableOp¬
3policy_network/conv1_d/convolution/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 25
3policy_network/conv1_d/convolution/ExpandDims_1/dim§
/policy_network/conv1_d/convolution/ExpandDims_1
ExpandDimsFpolicy_network/conv1_d/convolution/ExpandDims_1/ReadVariableOp:value:0<policy_network/conv1_d/convolution/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
: 21
/policy_network/conv1_d/convolution/ExpandDims_1§
"policy_network/conv1_d/convolutionConv2D6policy_network/conv1_d/convolution/ExpandDims:output:08policy_network/conv1_d/convolution/ExpandDims_1:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
paddingSAME*
strides
2$
"policy_network/conv1_d/convolutionç
*policy_network/conv1_d/convolution/SqueezeSqueeze+policy_network/conv1_d/convolution:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
squeeze_dims

ýÿÿÿÿÿÿÿÿ2,
*policy_network/conv1_d/convolution/SqueezeÑ
-policy_network/conv1_d/BiasAdd/ReadVariableOpReadVariableOp6policy_network_conv1_d_biasadd_readvariableop_resource*
_output_shapes
: *
dtype02/
-policy_network/conv1_d/BiasAdd/ReadVariableOpî
policy_network/conv1_d/BiasAddBiasAdd3policy_network/conv1_d/convolution/Squeeze:output:05policy_network/conv1_d/BiasAdd/ReadVariableOp:value:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2 
policy_network/conv1_d/BiasAdd¨
policy_network/sequential/ReluRelu'policy_network/conv1_d/BiasAdd:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2 
policy_network/sequential/Reluµ
3policy_network/conv1_d/convolution_1/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ýÿÿÿÿÿÿÿÿ25
3policy_network/conv1_d/convolution_1/ExpandDims/dim
/policy_network/conv1_d/convolution_1/ExpandDims
ExpandDims,policy_network/sequential/Relu:activations:0<policy_network/conv1_d/convolution_1/ExpandDims/dim:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 21
/policy_network/conv1_d/convolution_1/ExpandDims
@policy_network/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpReadVariableOpIpolicy_network_conv1_d_convolution_1_expanddims_1_readvariableop_resource*"
_output_shapes
:  *
dtype02B
@policy_network/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp°
5policy_network/conv1_d/convolution_1/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 27
5policy_network/conv1_d/convolution_1/ExpandDims_1/dim¯
1policy_network/conv1_d/convolution_1/ExpandDims_1
ExpandDimsHpolicy_network/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp:value:0>policy_network/conv1_d/convolution_1/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:  23
1policy_network/conv1_d/convolution_1/ExpandDims_1¯
$policy_network/conv1_d/convolution_1Conv2D8policy_network/conv1_d/convolution_1/ExpandDims:output:0:policy_network/conv1_d/convolution_1/ExpandDims_1:output:0*
T0*0
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
paddingSAME*
strides
2&
$policy_network/conv1_d/convolution_1í
,policy_network/conv1_d/convolution_1/SqueezeSqueeze-policy_network/conv1_d/convolution_1:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ *
squeeze_dims

ýÿÿÿÿÿÿÿÿ2.
,policy_network/conv1_d/convolution_1/Squeeze×
/policy_network/conv1_d/BiasAdd_1/ReadVariableOpReadVariableOp8policy_network_conv1_d_biasadd_1_readvariableop_resource*
_output_shapes
: *
dtype021
/policy_network/conv1_d/BiasAdd_1/ReadVariableOpö
 policy_network/conv1_d/BiasAdd_1BiasAdd5policy_network/conv1_d/convolution_1/Squeeze:output:07policy_network/conv1_d/BiasAdd_1/ReadVariableOp:value:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2"
 policy_network/conv1_d/BiasAdd_1®
 policy_network/sequential/Relu_1Relu)policy_network/conv1_d/BiasAdd_1:output:0*
T0*,
_output_shapes
:ÿÿÿÿÿÿÿÿÿñ 2"
 policy_network/sequential/Relu_1
policy_network/flatten/Shape_1Shape.policy_network/sequential/Relu_1:activations:0*
T0*
_output_shapes
:2 
policy_network/flatten/Shape_1¦
,policy_network/flatten/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB: 2.
,policy_network/flatten/strided_slice_1/stackª
.policy_network/flatten/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:20
.policy_network/flatten/strided_slice_1/stack_1ª
.policy_network/flatten/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:20
.policy_network/flatten/strided_slice_1/stack_2ö
&policy_network/flatten/strided_slice_1StridedSlice'policy_network/flatten/Shape_1:output:05policy_network/flatten/strided_slice_1/stack:output:07policy_network/flatten/strided_slice_1/stack_1:output:07policy_network/flatten/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2(
&policy_network/flatten/strided_slice_1
(policy_network/flatten/concat_1/values_1Const*
_output_shapes
:*
dtype0*
valueB: <2*
(policy_network/flatten/concat_1/values_1
$policy_network/flatten/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B : 2&
$policy_network/flatten/concat_1/axis
policy_network/flatten/concat_1ConcatV2/policy_network/flatten/strided_slice_1:output:01policy_network/flatten/concat_1/values_1:output:0-policy_network/flatten/concat_1/axis:output:0*
N*
T0*
_output_shapes
:2!
policy_network/flatten/concat_1Ü
 policy_network/flatten/Reshape_1Reshape.policy_network/sequential/Relu_1:activations:0(policy_network/flatten/concat_1:output:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ <2"
 policy_network/flatten/Reshape_1~
policy_network/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B :2
policy_network/concat_1/axisë
policy_network/concat_1ConcatV2policy_network/split:output:1)policy_network/flatten/Reshape_1:output:0%policy_network/concat_1/axis:output:0*
N*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ¾<2
policy_network/concat_1Ñ
+policy_network/linear/MatMul/ReadVariableOpReadVariableOp4policy_network_linear_matmul_readvariableop_resource* 
_output_shapes
:
¾<*
dtype02-
+policy_network/linear/MatMul/ReadVariableOpÐ
policy_network/linear/MatMulMatMul policy_network/concat_1:output:03policy_network/linear/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/MatMulÃ
(policy_network/linear/Add/ReadVariableOpReadVariableOp1policy_network_linear_add_readvariableop_resource*
_output_shapes	
:*
dtype02*
(policy_network/linear/Add/ReadVariableOpÊ
policy_network/linear/AddAdd&policy_network/linear/MatMul:product:00policy_network/linear/Add/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add
policy_network/sequential/EluElupolicy_network/linear/Add:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/sequential/Elu×
-policy_network/linear/MatMul_1/ReadVariableOpReadVariableOp6policy_network_linear_matmul_1_readvariableop_resource* 
_output_shapes
:
*
dtype02/
-policy_network/linear/MatMul_1/ReadVariableOpá
policy_network/linear/MatMul_1MatMul+policy_network/sequential/Elu:activations:05policy_network/linear/MatMul_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_1É
*policy_network/linear/Add_1/ReadVariableOpReadVariableOp3policy_network_linear_add_1_readvariableop_resource*
_output_shapes	
:*
dtype02,
*policy_network/linear/Add_1/ReadVariableOpÒ
policy_network/linear/Add_1Add(policy_network/linear/MatMul_1:product:02policy_network/linear/Add_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_1
policy_network/sequential/Elu_1Elupolicy_network/linear/Add_1:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/sequential/Elu_1×
-policy_network/linear/MatMul_2/ReadVariableOpReadVariableOp6policy_network_linear_matmul_2_readvariableop_resource* 
_output_shapes
:
*
dtype02/
-policy_network/linear/MatMul_2/ReadVariableOpã
policy_network/linear/MatMul_2MatMul-policy_network/sequential/Elu_1:activations:05policy_network/linear/MatMul_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_2É
*policy_network/linear/Add_2/ReadVariableOpReadVariableOp3policy_network_linear_add_2_readvariableop_resource*
_output_shapes	
:*
dtype02,
*policy_network/linear/Add_2/ReadVariableOpÒ
policy_network/linear/Add_2Add(policy_network/linear/MatMul_2:product:02policy_network/linear/Add_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_2
policy_network/sequential/Elu_2Elupolicy_network/linear/Add_2:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/sequential/Elu_2×
-policy_network/linear/MatMul_3/ReadVariableOpReadVariableOp6policy_network_linear_matmul_3_readvariableop_resource* 
_output_shapes
:
*
dtype02/
-policy_network/linear/MatMul_3/ReadVariableOpã
policy_network/linear/MatMul_3MatMul-policy_network/sequential/Elu_2:activations:05policy_network/linear/MatMul_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_3É
*policy_network/linear/Add_3/ReadVariableOpReadVariableOp3policy_network_linear_add_3_readvariableop_resource*
_output_shapes	
:*
dtype02,
*policy_network/linear/Add_3/ReadVariableOpÒ
policy_network/linear/Add_3Add(policy_network/linear/MatMul_3:product:02policy_network/linear/Add_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_3
policy_network/sequential/Elu_3Elupolicy_network/linear/Add_3:z:0*
T0*(
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/sequential/Elu_3Ö
-policy_network/linear/MatMul_4/ReadVariableOpReadVariableOp6policy_network_linear_matmul_4_readvariableop_resource*
_output_shapes
:	*
dtype02/
-policy_network/linear/MatMul_4/ReadVariableOpâ
policy_network/linear/MatMul_4MatMul-policy_network/sequential/Elu_3:activations:05policy_network/linear/MatMul_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2 
policy_network/linear/MatMul_4È
*policy_network/linear/Add_4/ReadVariableOpReadVariableOp3policy_network_linear_add_4_readvariableop_resource*
_output_shapes
:*
dtype02,
*policy_network/linear/Add_4/ReadVariableOpÑ
policy_network/linear/Add_4Add(policy_network/linear/MatMul_4:product:02policy_network/linear/Add_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2
policy_network/linear/Add_4
 policy_network/tanh_to_spec/TanhTanhpolicy_network/linear/Add_4:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2"
 policy_network/tanh_to_spec/Tanh
!policy_network/tanh_to_spec/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *  ?2#
!policy_network/tanh_to_spec/add/yÏ
policy_network/tanh_to_spec/addAddV2$policy_network/tanh_to_spec/Tanh:y:0*policy_network/tanh_to_spec/add/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/tanh_to_spec/add
!policy_network/tanh_to_spec/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2#
!policy_network/tanh_to_spec/mul/xÌ
policy_network/tanh_to_spec/mulMul*policy_network/tanh_to_spec/mul/x:output:0#policy_network/tanh_to_spec/add:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2!
policy_network/tanh_to_spec/mul
#policy_network/tanh_to_spec/mul_1/yConst*
_output_shapes
:*
dtype0*
valueB"  ?   @2%
#policy_network/tanh_to_spec/mul_1/yÒ
!policy_network/tanh_to_spec/mul_1Mul#policy_network/tanh_to_spec/mul:z:0,policy_network/tanh_to_spec/mul_1/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2#
!policy_network/tanh_to_spec/mul_1
#policy_network/tanh_to_spec/add_1/yConst*
_output_shapes
:*
dtype0*
valueB"      ¿2%
#policy_network/tanh_to_spec/add_1/yÖ
!policy_network/tanh_to_spec/add_1AddV2%policy_network/tanh_to_spec/mul_1:z:0,policy_network/tanh_to_spec/add_1/y:output:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2#
!policy_network/tanh_to_spec/add_1y
IdentityIdentity%policy_network/tanh_to_spec/add_1:z:0*
T0*'
_output_shapes
:ÿÿÿÿÿÿÿÿÿ2

Identity"
identityIdentity:output:0*_
_input_shapesN
L:ÿÿÿÿÿÿÿÿÿâ:::::::::::::::P L
(
_output_shapes
:ÿÿÿÿÿÿÿÿÿâ
 
_user_specified_nameargs_0"¸J
saver_filename:0StatefulPartitionedCall:0StatefulPartitionedCall_18"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp:û
l

_variables
_trainable_variables

signatures
__call__
_module"
acme_snapshot

0
1
2
3
4
	5

6
7
8
9
10
11
12
13"
trackable_tuple_wrapper

0
1
2
3
4
	5

6
7
8
9
10
11
12
13"
trackable_tuple_wrapper
"
signature_map
&:$ 2policy_network/conv1_d/b
.:, 2policy_network/conv1_d/w
&:$ 2policy_network/conv1_d/b
.:,  2policy_network/conv1_d/w
&:$2policy_network/linear/b
+:)
¾<2policy_network/linear/w
&:$2policy_network/linear/b
+:)
2policy_network/linear/w
&:$2policy_network/linear/b
+:)
2policy_network/linear/w
&:$2policy_network/linear/b
+:)
2policy_network/linear/w
%:#2policy_network/linear/b
*:(	2policy_network/linear/w
Ä2Á
__inference___call___27733¢
²
FullArgSpec
args
jself
varargsjargs
varkwjkwargs
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 
À2½
 __inference_wrapped_module_27839
²
FullArgSpec
args 
varargsjargs
varkwjkwargs
defaults
 

kwonlyargs 
kwonlydefaults
 
annotationsª *
 z
__inference___call___27733\	
0¢-
&¢#
!
args_0ÿÿÿÿÿÿÿÿÿâ
ª "ÿÿÿÿÿÿÿÿÿ
 __inference_wrapped_module_27839\	
0¢-
&¢#
!
args_0ÿÿÿÿÿÿÿÿÿâ
ª "ÿÿÿÿÿÿÿÿÿ