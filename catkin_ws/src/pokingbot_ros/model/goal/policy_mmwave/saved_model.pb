ΑΔ
τ
Ι

B
AssignVariableOp
resource
value"dtype"
dtypetype
8
Const
output"dtype"
valuetensor"
dtypetype
.
Identity

input"T
output"T"	
Ttype
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
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
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0
?
Select
	condition

t"T
e"T
output"T"	
Ttype
H
ShardedFilename
basename	
shard

num_shards
filename
Ύ
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
@
StaticRegexFullMatch	
input

output
"
patternstring
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 

VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 "serve*2.4.12v2.4.0-49-g85c8b2a817f8ͺ

policy_network_cnn/conv1_d/bVarHandleOp*
_output_shapes
: *
dtype0*
shape: *-
shared_namepolicy_network_cnn/conv1_d/b

0policy_network_cnn/conv1_d/b/Read/ReadVariableOpReadVariableOppolicy_network_cnn/conv1_d/b*
_output_shapes
: *
dtype0

policy_network_cnn/conv1_d/wVarHandleOp*
_output_shapes
: *
dtype0*
shape: *-
shared_namepolicy_network_cnn/conv1_d/w

0policy_network_cnn/conv1_d/w/Read/ReadVariableOpReadVariableOppolicy_network_cnn/conv1_d/w*"
_output_shapes
: *
dtype0

policy_network_cnn/conv1_d/b_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: */
shared_name policy_network_cnn/conv1_d/b_1

2policy_network_cnn/conv1_d/b_1/Read/ReadVariableOpReadVariableOppolicy_network_cnn/conv1_d/b_1*
_output_shapes
: *
dtype0

policy_network_cnn/conv1_d/w_1VarHandleOp*
_output_shapes
: *
dtype0*
shape:  */
shared_name policy_network_cnn/conv1_d/w_1

2policy_network_cnn/conv1_d/w_1/Read/ReadVariableOpReadVariableOppolicy_network_cnn/conv1_d/w_1*"
_output_shapes
:  *
dtype0

policy_network_cnn/linear/bVarHandleOp*
_output_shapes
: *
dtype0*
shape:*,
shared_namepolicy_network_cnn/linear/b

/policy_network_cnn/linear/b/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/b*
_output_shapes	
:*
dtype0

policy_network_cnn/linear/wVarHandleOp*
_output_shapes
: *
dtype0*
shape:
Ύ<*,
shared_namepolicy_network_cnn/linear/w

/policy_network_cnn/linear/w/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/w* 
_output_shapes
:
Ύ<*
dtype0

policy_network_cnn/linear/b_1VarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namepolicy_network_cnn/linear/b_1

1policy_network_cnn/linear/b_1/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/b_1*
_output_shapes	
:*
dtype0

policy_network_cnn/linear/w_1VarHandleOp*
_output_shapes
: *
dtype0*
shape:
*.
shared_namepolicy_network_cnn/linear/w_1

1policy_network_cnn/linear/w_1/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/w_1* 
_output_shapes
:
*
dtype0

policy_network_cnn/linear/b_2VarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namepolicy_network_cnn/linear/b_2

1policy_network_cnn/linear/b_2/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/b_2*
_output_shapes	
:*
dtype0

policy_network_cnn/linear/w_2VarHandleOp*
_output_shapes
: *
dtype0*
shape:
*.
shared_namepolicy_network_cnn/linear/w_2

1policy_network_cnn/linear/w_2/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/w_2* 
_output_shapes
:
*
dtype0

policy_network_cnn/linear/b_3VarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namepolicy_network_cnn/linear/b_3

1policy_network_cnn/linear/b_3/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/b_3*
_output_shapes	
:*
dtype0

policy_network_cnn/linear/w_3VarHandleOp*
_output_shapes
: *
dtype0*
shape:
*.
shared_namepolicy_network_cnn/linear/w_3

1policy_network_cnn/linear/w_3/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/w_3* 
_output_shapes
:
*
dtype0

policy_network_cnn/linear/b_4VarHandleOp*
_output_shapes
: *
dtype0*
shape:*.
shared_namepolicy_network_cnn/linear/b_4

1policy_network_cnn/linear/b_4/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/b_4*
_output_shapes
:*
dtype0

policy_network_cnn/linear/w_4VarHandleOp*
_output_shapes
: *
dtype0*
shape:	*.
shared_namepolicy_network_cnn/linear/w_4

1policy_network_cnn/linear/w_4/Read/ReadVariableOpReadVariableOppolicy_network_cnn/linear/w_4*
_output_shapes
:	*
dtype0

NoOpNoOp
ο
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*ͺ
value B B
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
YW
VARIABLE_VALUEpolicy_network_cnn/conv1_d/b'_variables/0/.ATTRIBUTES/VARIABLE_VALUE
YW
VARIABLE_VALUEpolicy_network_cnn/conv1_d/w'_variables/1/.ATTRIBUTES/VARIABLE_VALUE
[Y
VARIABLE_VALUEpolicy_network_cnn/conv1_d/b_1'_variables/2/.ATTRIBUTES/VARIABLE_VALUE
[Y
VARIABLE_VALUEpolicy_network_cnn/conv1_d/w_1'_variables/3/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEpolicy_network_cnn/linear/b'_variables/4/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEpolicy_network_cnn/linear/w'_variables/5/.ATTRIBUTES/VARIABLE_VALUE
ZX
VARIABLE_VALUEpolicy_network_cnn/linear/b_1'_variables/6/.ATTRIBUTES/VARIABLE_VALUE
ZX
VARIABLE_VALUEpolicy_network_cnn/linear/w_1'_variables/7/.ATTRIBUTES/VARIABLE_VALUE
ZX
VARIABLE_VALUEpolicy_network_cnn/linear/b_2'_variables/8/.ATTRIBUTES/VARIABLE_VALUE
ZX
VARIABLE_VALUEpolicy_network_cnn/linear/w_2'_variables/9/.ATTRIBUTES/VARIABLE_VALUE
[Y
VARIABLE_VALUEpolicy_network_cnn/linear/b_3(_variables/10/.ATTRIBUTES/VARIABLE_VALUE
[Y
VARIABLE_VALUEpolicy_network_cnn/linear/w_3(_variables/11/.ATTRIBUTES/VARIABLE_VALUE
[Y
VARIABLE_VALUEpolicy_network_cnn/linear/b_4(_variables/12/.ATTRIBUTES/VARIABLE_VALUE
[Y
VARIABLE_VALUEpolicy_network_cnn/linear/w_4(_variables/13/.ATTRIBUTES/VARIABLE_VALUE
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
σ
StatefulPartitionedCallStatefulPartitionedCallsaver_filename0policy_network_cnn/conv1_d/b/Read/ReadVariableOp0policy_network_cnn/conv1_d/w/Read/ReadVariableOp2policy_network_cnn/conv1_d/b_1/Read/ReadVariableOp2policy_network_cnn/conv1_d/w_1/Read/ReadVariableOp/policy_network_cnn/linear/b/Read/ReadVariableOp/policy_network_cnn/linear/w/Read/ReadVariableOp1policy_network_cnn/linear/b_1/Read/ReadVariableOp1policy_network_cnn/linear/w_1/Read/ReadVariableOp1policy_network_cnn/linear/b_2/Read/ReadVariableOp1policy_network_cnn/linear/w_2/Read/ReadVariableOp1policy_network_cnn/linear/b_3/Read/ReadVariableOp1policy_network_cnn/linear/w_3/Read/ReadVariableOp1policy_network_cnn/linear/b_4/Read/ReadVariableOp1policy_network_cnn/linear/w_4/Read/ReadVariableOpConst*
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
 __inference__traced_save_2911288
Ψ
StatefulPartitionedCall_1StatefulPartitionedCallsaver_filenamepolicy_network_cnn/conv1_d/bpolicy_network_cnn/conv1_d/wpolicy_network_cnn/conv1_d/b_1policy_network_cnn/conv1_d/w_1policy_network_cnn/linear/bpolicy_network_cnn/linear/wpolicy_network_cnn/linear/b_1policy_network_cnn/linear/w_1policy_network_cnn/linear/b_2policy_network_cnn/linear/w_2policy_network_cnn/linear/b_3policy_network_cnn/linear/w_3policy_network_cnn/linear/b_4policy_network_cnn/linear/w_4*
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
#__inference__traced_restore_2911340£ξ
½
Ο
"__inference_wrapped_module_2911085

args_0O
Kpolicy_network_cnn_conv1_d_convolution_expanddims_1_readvariableop_resource>
:policy_network_cnn_conv1_d_biasadd_readvariableop_resourceQ
Mpolicy_network_cnn_conv1_d_convolution_1_expanddims_1_readvariableop_resource@
<policy_network_cnn_conv1_d_biasadd_1_readvariableop_resource<
8policy_network_cnn_linear_matmul_readvariableop_resource9
5policy_network_cnn_linear_add_readvariableop_resource>
:policy_network_cnn_linear_matmul_1_readvariableop_resource;
7policy_network_cnn_linear_add_1_readvariableop_resource>
:policy_network_cnn_linear_matmul_2_readvariableop_resource;
7policy_network_cnn_linear_add_2_readvariableop_resource>
:policy_network_cnn_linear_matmul_3_readvariableop_resource;
7policy_network_cnn_linear_add_3_readvariableop_resource>
:policy_network_cnn_linear_matmul_4_readvariableop_resource;
7policy_network_cnn_linear_add_4_readvariableop_resource
identity

identity_1’1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp’3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp’Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp’Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp’,policy_network_cnn/linear/Add/ReadVariableOp’.policy_network_cnn/linear/Add_1/ReadVariableOp’.policy_network_cnn/linear/Add_2/ReadVariableOp’.policy_network_cnn/linear/Add_3/ReadVariableOp’.policy_network_cnn/linear/Add_4/ReadVariableOp’/policy_network_cnn/linear/MatMul/ReadVariableOp’1policy_network_cnn/linear/MatMul_1/ReadVariableOp’1policy_network_cnn/linear/MatMul_2/ReadVariableOp’1policy_network_cnn/linear/MatMul_3/ReadVariableOp’1policy_network_cnn/linear/MatMul_4/ReadVariableOpz
 policy_network_cnn/flatten/ShapeShapeargs_0*
T0*
_output_shapes
:2"
 policy_network_cnn/flatten/Shapeͺ
.policy_network_cnn/flatten/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 20
.policy_network_cnn/flatten/strided_slice/stack?
0policy_network_cnn/flatten/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/flatten/strided_slice/stack_1?
0policy_network_cnn/flatten/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/flatten/strided_slice/stack_2
(policy_network_cnn/flatten/strided_sliceStridedSlice)policy_network_cnn/flatten/Shape:output:07policy_network_cnn/flatten/strided_slice/stack:output:09policy_network_cnn/flatten/strided_slice/stack_1:output:09policy_network_cnn/flatten/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2*
(policy_network_cnn/flatten/strided_slice£
*policy_network_cnn/flatten/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB:β2,
*policy_network_cnn/flatten/concat/values_1
&policy_network_cnn/flatten/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2(
&policy_network_cnn/flatten/concat/axis
!policy_network_cnn/flatten/concatConcatV21policy_network_cnn/flatten/strided_slice:output:03policy_network_cnn/flatten/concat/values_1:output:0/policy_network_cnn/flatten/concat/axis:output:0*
N*
T0*
_output_shapes
:2#
!policy_network_cnn/flatten/concatΊ
"policy_network_cnn/flatten/ReshapeReshapeargs_0*policy_network_cnn/flatten/concat:output:0*
T0*(
_output_shapes
:?????????β2$
"policy_network_cnn/flatten/Reshape
$policy_network_cnn/concat/concat_dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2&
$policy_network_cnn/concat/concat_dim°
 policy_network_cnn/concat/concatIdentity+policy_network_cnn/flatten/Reshape:output:0*
T0*(
_output_shapes
:?????????β2"
 policy_network_cnn/concat/concat
policy_network_cnn/ConstConst*
_output_shapes
:*
dtype0*
valueB"Δ     2
policy_network_cnn/Const
"policy_network_cnn/split/split_dimConst*
_output_shapes
: *
dtype0*
value	B :2$
"policy_network_cnn/split/split_dim
policy_network_cnn/splitSplitV)policy_network_cnn/concat/concat:output:0!policy_network_cnn/Const:output:0+policy_network_cnn/split/split_dim:output:0*
T0*

Tlen0*;
_output_shapes)
':?????????Δ:?????????*
	num_split2
policy_network_cnn/split
 policy_network_cnn/reshape/ShapeShape!policy_network_cnn/split:output:0*
T0*
_output_shapes
:2"
 policy_network_cnn/reshape/Shapeͺ
.policy_network_cnn/reshape/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 20
.policy_network_cnn/reshape/strided_slice/stack?
0policy_network_cnn/reshape/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/reshape/strided_slice/stack_1?
0policy_network_cnn/reshape/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/reshape/strided_slice/stack_2
(policy_network_cnn/reshape/strided_sliceStridedSlice)policy_network_cnn/reshape/Shape:output:07policy_network_cnn/reshape/strided_slice/stack:output:09policy_network_cnn/reshape/strided_slice/stack_1:output:09policy_network_cnn/reshape/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2*
(policy_network_cnn/reshape/strided_slice©
*policy_network_cnn/reshape/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB"ρ      2,
*policy_network_cnn/reshape/concat/values_1
&policy_network_cnn/reshape/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2(
&policy_network_cnn/reshape/concat/axis
!policy_network_cnn/reshape/concatConcatV21policy_network_cnn/reshape/strided_slice:output:03policy_network_cnn/reshape/concat/values_1:output:0/policy_network_cnn/reshape/concat/axis:output:0*
N*
T0*
_output_shapes
:2#
!policy_network_cnn/reshape/concatΩ
"policy_network_cnn/reshape/ReshapeReshape!policy_network_cnn/split:output:0*policy_network_cnn/reshape/concat:output:0*
T0*,
_output_shapes
:?????????ρ2$
"policy_network_cnn/reshape/ReshapeΉ
5policy_network_cnn/conv1_d/convolution/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ύ????????27
5policy_network_cnn/conv1_d/convolution/ExpandDims/dim
1policy_network_cnn/conv1_d/convolution/ExpandDims
ExpandDims+policy_network_cnn/reshape/Reshape:output:0>policy_network_cnn/conv1_d/convolution/ExpandDims/dim:output:0*
T0*0
_output_shapes
:?????????ρ23
1policy_network_cnn/conv1_d/convolution/ExpandDims
Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpReadVariableOpKpolicy_network_cnn_conv1_d_convolution_expanddims_1_readvariableop_resource*"
_output_shapes
: *
dtype02D
Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp΄
7policy_network_cnn/conv1_d/convolution/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 29
7policy_network_cnn/conv1_d/convolution/ExpandDims_1/dim·
3policy_network_cnn/conv1_d/convolution/ExpandDims_1
ExpandDimsJpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp:value:0@policy_network_cnn/conv1_d/convolution/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
: 25
3policy_network_cnn/conv1_d/convolution/ExpandDims_1·
&policy_network_cnn/conv1_d/convolutionConv2D:policy_network_cnn/conv1_d/convolution/ExpandDims:output:0<policy_network_cnn/conv1_d/convolution/ExpandDims_1:output:0*
T0*0
_output_shapes
:?????????ρ *
paddingSAME*
strides
2(
&policy_network_cnn/conv1_d/convolutionσ
.policy_network_cnn/conv1_d/convolution/SqueezeSqueeze/policy_network_cnn/conv1_d/convolution:output:0*
T0*,
_output_shapes
:?????????ρ *
squeeze_dims

ύ????????20
.policy_network_cnn/conv1_d/convolution/Squeezeέ
1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOpReadVariableOp:policy_network_cnn_conv1_d_biasadd_readvariableop_resource*
_output_shapes
: *
dtype023
1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOpώ
"policy_network_cnn/conv1_d/BiasAddBiasAdd7policy_network_cnn/conv1_d/convolution/Squeeze:output:09policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp:value:0*
T0*,
_output_shapes
:?????????ρ 2$
"policy_network_cnn/conv1_d/BiasAdd΄
"policy_network_cnn/sequential/ReluRelu+policy_network_cnn/conv1_d/BiasAdd:output:0*
T0*,
_output_shapes
:?????????ρ 2$
"policy_network_cnn/sequential/Relu½
7policy_network_cnn/conv1_d/convolution_1/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ύ????????29
7policy_network_cnn/conv1_d/convolution_1/ExpandDims/dim§
3policy_network_cnn/conv1_d/convolution_1/ExpandDims
ExpandDims0policy_network_cnn/sequential/Relu:activations:0@policy_network_cnn/conv1_d/convolution_1/ExpandDims/dim:output:0*
T0*0
_output_shapes
:?????????ρ 25
3policy_network_cnn/conv1_d/convolution_1/ExpandDims
Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpReadVariableOpMpolicy_network_cnn_conv1_d_convolution_1_expanddims_1_readvariableop_resource*"
_output_shapes
:  *
dtype02F
Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpΈ
9policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2;
9policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/dimΏ
5policy_network_cnn/conv1_d/convolution_1/ExpandDims_1
ExpandDimsLpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp:value:0Bpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:  27
5policy_network_cnn/conv1_d/convolution_1/ExpandDims_1Ώ
(policy_network_cnn/conv1_d/convolution_1Conv2D<policy_network_cnn/conv1_d/convolution_1/ExpandDims:output:0>policy_network_cnn/conv1_d/convolution_1/ExpandDims_1:output:0*
T0*0
_output_shapes
:?????????ρ *
paddingSAME*
strides
2*
(policy_network_cnn/conv1_d/convolution_1ω
0policy_network_cnn/conv1_d/convolution_1/SqueezeSqueeze1policy_network_cnn/conv1_d/convolution_1:output:0*
T0*,
_output_shapes
:?????????ρ *
squeeze_dims

ύ????????22
0policy_network_cnn/conv1_d/convolution_1/Squeezeγ
3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOpReadVariableOp<policy_network_cnn_conv1_d_biasadd_1_readvariableop_resource*
_output_shapes
: *
dtype025
3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp
$policy_network_cnn/conv1_d/BiasAdd_1BiasAdd9policy_network_cnn/conv1_d/convolution_1/Squeeze:output:0;policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp:value:0*
T0*,
_output_shapes
:?????????ρ 2&
$policy_network_cnn/conv1_d/BiasAdd_1Ί
$policy_network_cnn/sequential/Relu_1Relu-policy_network_cnn/conv1_d/BiasAdd_1:output:0*
T0*,
_output_shapes
:?????????ρ 2&
$policy_network_cnn/sequential/Relu_1ͺ
"policy_network_cnn/flatten/Shape_1Shape2policy_network_cnn/sequential/Relu_1:activations:0*
T0*
_output_shapes
:2$
"policy_network_cnn/flatten/Shape_1?
0policy_network_cnn/flatten/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB: 22
0policy_network_cnn/flatten/strided_slice_1/stack²
2policy_network_cnn/flatten/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:24
2policy_network_cnn/flatten/strided_slice_1/stack_1²
2policy_network_cnn/flatten/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:24
2policy_network_cnn/flatten/strided_slice_1/stack_2
*policy_network_cnn/flatten/strided_slice_1StridedSlice+policy_network_cnn/flatten/Shape_1:output:09policy_network_cnn/flatten/strided_slice_1/stack:output:0;policy_network_cnn/flatten/strided_slice_1/stack_1:output:0;policy_network_cnn/flatten/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2,
*policy_network_cnn/flatten/strided_slice_1§
,policy_network_cnn/flatten/concat_1/values_1Const*
_output_shapes
:*
dtype0*
valueB: <2.
,policy_network_cnn/flatten/concat_1/values_1
(policy_network_cnn/flatten/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B : 2*
(policy_network_cnn/flatten/concat_1/axis£
#policy_network_cnn/flatten/concat_1ConcatV23policy_network_cnn/flatten/strided_slice_1:output:05policy_network_cnn/flatten/concat_1/values_1:output:01policy_network_cnn/flatten/concat_1/axis:output:0*
N*
T0*
_output_shapes
:2%
#policy_network_cnn/flatten/concat_1μ
$policy_network_cnn/flatten/Reshape_1Reshape2policy_network_cnn/sequential/Relu_1:activations:0,policy_network_cnn/flatten/concat_1:output:0*
T0*(
_output_shapes
:????????? <2&
$policy_network_cnn/flatten/Reshape_1
 policy_network_cnn/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B :2"
 policy_network_cnn/concat_1/axis?
policy_network_cnn/concat_1ConcatV2!policy_network_cnn/split:output:1-policy_network_cnn/flatten/Reshape_1:output:0)policy_network_cnn/concat_1/axis:output:0*
N*
T0*(
_output_shapes
:?????????Ύ<2
policy_network_cnn/concat_1έ
/policy_network_cnn/linear/MatMul/ReadVariableOpReadVariableOp8policy_network_cnn_linear_matmul_readvariableop_resource* 
_output_shapes
:
Ύ<*
dtype021
/policy_network_cnn/linear/MatMul/ReadVariableOpΰ
 policy_network_cnn/linear/MatMulMatMul$policy_network_cnn/concat_1:output:07policy_network_cnn/linear/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2"
 policy_network_cnn/linear/MatMulΟ
,policy_network_cnn/linear/Add/ReadVariableOpReadVariableOp5policy_network_cnn_linear_add_readvariableop_resource*
_output_shapes	
:*
dtype02.
,policy_network_cnn/linear/Add/ReadVariableOpΪ
policy_network_cnn/linear/AddAdd*policy_network_cnn/linear/MatMul:product:04policy_network_cnn/linear/Add/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2
policy_network_cnn/linear/Add£
!policy_network_cnn/sequential/EluElu!policy_network_cnn/linear/Add:z:0*
T0*(
_output_shapes
:?????????2#
!policy_network_cnn/sequential/Eluγ
1policy_network_cnn/linear/MatMul_1/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_1_readvariableop_resource* 
_output_shapes
:
*
dtype023
1policy_network_cnn/linear/MatMul_1/ReadVariableOpρ
"policy_network_cnn/linear/MatMul_1MatMul/policy_network_cnn/sequential/Elu:activations:09policy_network_cnn/linear/MatMul_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_1Υ
.policy_network_cnn/linear/Add_1/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_1_readvariableop_resource*
_output_shapes	
:*
dtype020
.policy_network_cnn/linear/Add_1/ReadVariableOpβ
policy_network_cnn/linear/Add_1Add,policy_network_cnn/linear/MatMul_1:product:06policy_network_cnn/linear/Add_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_1©
#policy_network_cnn/sequential/Elu_1Elu#policy_network_cnn/linear/Add_1:z:0*
T0*(
_output_shapes
:?????????2%
#policy_network_cnn/sequential/Elu_1γ
1policy_network_cnn/linear/MatMul_2/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_2_readvariableop_resource* 
_output_shapes
:
*
dtype023
1policy_network_cnn/linear/MatMul_2/ReadVariableOpσ
"policy_network_cnn/linear/MatMul_2MatMul1policy_network_cnn/sequential/Elu_1:activations:09policy_network_cnn/linear/MatMul_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_2Υ
.policy_network_cnn/linear/Add_2/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_2_readvariableop_resource*
_output_shapes	
:*
dtype020
.policy_network_cnn/linear/Add_2/ReadVariableOpβ
policy_network_cnn/linear/Add_2Add,policy_network_cnn/linear/MatMul_2:product:06policy_network_cnn/linear/Add_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_2©
#policy_network_cnn/sequential/Elu_2Elu#policy_network_cnn/linear/Add_2:z:0*
T0*(
_output_shapes
:?????????2%
#policy_network_cnn/sequential/Elu_2γ
1policy_network_cnn/linear/MatMul_3/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_3_readvariableop_resource* 
_output_shapes
:
*
dtype023
1policy_network_cnn/linear/MatMul_3/ReadVariableOpσ
"policy_network_cnn/linear/MatMul_3MatMul1policy_network_cnn/sequential/Elu_2:activations:09policy_network_cnn/linear/MatMul_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_3Υ
.policy_network_cnn/linear/Add_3/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_3_readvariableop_resource*
_output_shapes	
:*
dtype020
.policy_network_cnn/linear/Add_3/ReadVariableOpβ
policy_network_cnn/linear/Add_3Add,policy_network_cnn/linear/MatMul_3:product:06policy_network_cnn/linear/Add_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_3©
#policy_network_cnn/sequential/Elu_3Elu#policy_network_cnn/linear/Add_3:z:0*
T0*(
_output_shapes
:?????????2%
#policy_network_cnn/sequential/Elu_3β
1policy_network_cnn/linear/MatMul_4/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_4_readvariableop_resource*
_output_shapes
:	*
dtype023
1policy_network_cnn/linear/MatMul_4/ReadVariableOpς
"policy_network_cnn/linear/MatMul_4MatMul1policy_network_cnn/sequential/Elu_3:activations:09policy_network_cnn/linear/MatMul_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_4Τ
.policy_network_cnn/linear/Add_4/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_4_readvariableop_resource*
_output_shapes
:*
dtype020
.policy_network_cnn/linear/Add_4/ReadVariableOpα
policy_network_cnn/linear/Add_4Add,policy_network_cnn/linear/MatMul_4:product:06policy_network_cnn/linear/Add_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_4«
$policy_network_cnn/tanh_to_spec/TanhTanh#policy_network_cnn/linear/Add_4:z:0*
T0*'
_output_shapes
:?????????2&
$policy_network_cnn/tanh_to_spec/Tanh
%policy_network_cnn/tanh_to_spec/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *  ?2'
%policy_network_cnn/tanh_to_spec/add/yί
#policy_network_cnn/tanh_to_spec/addAddV2(policy_network_cnn/tanh_to_spec/Tanh:y:0.policy_network_cnn/tanh_to_spec/add/y:output:0*
T0*'
_output_shapes
:?????????2%
#policy_network_cnn/tanh_to_spec/add
%policy_network_cnn/tanh_to_spec/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2'
%policy_network_cnn/tanh_to_spec/mul/xά
#policy_network_cnn/tanh_to_spec/mulMul.policy_network_cnn/tanh_to_spec/mul/x:output:0'policy_network_cnn/tanh_to_spec/add:z:0*
T0*'
_output_shapes
:?????????2%
#policy_network_cnn/tanh_to_spec/mul£
'policy_network_cnn/tanh_to_spec/mul_1/yConst*
_output_shapes
:*
dtype0*
valueB"  ?   @2)
'policy_network_cnn/tanh_to_spec/mul_1/yβ
%policy_network_cnn/tanh_to_spec/mul_1Mul'policy_network_cnn/tanh_to_spec/mul:z:00policy_network_cnn/tanh_to_spec/mul_1/y:output:0*
T0*'
_output_shapes
:?????????2'
%policy_network_cnn/tanh_to_spec/mul_1£
'policy_network_cnn/tanh_to_spec/add_1/yConst*
_output_shapes
:*
dtype0*
valueB"      Ώ2)
'policy_network_cnn/tanh_to_spec/add_1/yζ
%policy_network_cnn/tanh_to_spec/add_1AddV2)policy_network_cnn/tanh_to_spec/mul_1:z:00policy_network_cnn/tanh_to_spec/add_1/y:output:0*
T0*'
_output_shapes
:?????????2'
%policy_network_cnn/tanh_to_spec/add_1ν
IdentityIdentity-policy_network_cnn/flatten/Reshape_1:output:02^policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp4^policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOpC^policy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpE^policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp-^policy_network_cnn/linear/Add/ReadVariableOp/^policy_network_cnn/linear/Add_1/ReadVariableOp/^policy_network_cnn/linear/Add_2/ReadVariableOp/^policy_network_cnn/linear/Add_3/ReadVariableOp/^policy_network_cnn/linear/Add_4/ReadVariableOp0^policy_network_cnn/linear/MatMul/ReadVariableOp2^policy_network_cnn/linear/MatMul_1/ReadVariableOp2^policy_network_cnn/linear/MatMul_2/ReadVariableOp2^policy_network_cnn/linear/MatMul_3/ReadVariableOp2^policy_network_cnn/linear/MatMul_4/ReadVariableOp*
T0*(
_output_shapes
:????????? <2

Identityμ

Identity_1Identity)policy_network_cnn/tanh_to_spec/add_1:z:02^policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp4^policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOpC^policy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpE^policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp-^policy_network_cnn/linear/Add/ReadVariableOp/^policy_network_cnn/linear/Add_1/ReadVariableOp/^policy_network_cnn/linear/Add_2/ReadVariableOp/^policy_network_cnn/linear/Add_3/ReadVariableOp/^policy_network_cnn/linear/Add_4/ReadVariableOp0^policy_network_cnn/linear/MatMul/ReadVariableOp2^policy_network_cnn/linear/MatMul_1/ReadVariableOp2^policy_network_cnn/linear/MatMul_2/ReadVariableOp2^policy_network_cnn/linear/MatMul_3/ReadVariableOp2^policy_network_cnn/linear/MatMul_4/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity_1"
identityIdentity:output:0"!

identity_1Identity_1:output:0*_
_input_shapesN
L:?????????β::::::::::::::2f
1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp2j
3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp2
Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpBpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp2
Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpDpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp2\
,policy_network_cnn/linear/Add/ReadVariableOp,policy_network_cnn/linear/Add/ReadVariableOp2`
.policy_network_cnn/linear/Add_1/ReadVariableOp.policy_network_cnn/linear/Add_1/ReadVariableOp2`
.policy_network_cnn/linear/Add_2/ReadVariableOp.policy_network_cnn/linear/Add_2/ReadVariableOp2`
.policy_network_cnn/linear/Add_3/ReadVariableOp.policy_network_cnn/linear/Add_3/ReadVariableOp2`
.policy_network_cnn/linear/Add_4/ReadVariableOp.policy_network_cnn/linear/Add_4/ReadVariableOp2b
/policy_network_cnn/linear/MatMul/ReadVariableOp/policy_network_cnn/linear/MatMul/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_1/ReadVariableOp1policy_network_cnn/linear/MatMul_1/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_2/ReadVariableOp1policy_network_cnn/linear/MatMul_2/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_3/ReadVariableOp1policy_network_cnn/linear/MatMul_3/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_4/ReadVariableOp1policy_network_cnn/linear/MatMul_4/ReadVariableOp:P L
(
_output_shapes
:?????????β
 
_user_specified_nameargs_0

Έ
__inference___call___2911118

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
identity

identity_1’StatefulPartitionedCall
StatefulPartitionedCallStatefulPartitionedCallargs_0unknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4	unknown_5	unknown_6	unknown_7	unknown_8	unknown_9
unknown_10
unknown_11
unknown_12*
Tin
2*
Tout
2*
_collective_manager_ids
 *;
_output_shapes)
':????????? <:?????????*0
_read_only_resource_inputs
	
*2
config_proto" 

CPU

GPU2 *0J 8 *+
f&R$
"__inference_wrapped_module_29110852
StatefulPartitionedCall
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*(
_output_shapes
:????????? <2

Identity

Identity_1Identity StatefulPartitionedCall:output:1^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity_1"
identityIdentity:output:0"!

identity_1Identity_1:output:0*_
_input_shapesN
L:?????????β::::::::::::::22
StatefulPartitionedCallStatefulPartitionedCall:P L
(
_output_shapes
:?????????β
 
_user_specified_nameargs_0
½
Ο
"__inference_wrapped_module_2911225

args_0O
Kpolicy_network_cnn_conv1_d_convolution_expanddims_1_readvariableop_resource>
:policy_network_cnn_conv1_d_biasadd_readvariableop_resourceQ
Mpolicy_network_cnn_conv1_d_convolution_1_expanddims_1_readvariableop_resource@
<policy_network_cnn_conv1_d_biasadd_1_readvariableop_resource<
8policy_network_cnn_linear_matmul_readvariableop_resource9
5policy_network_cnn_linear_add_readvariableop_resource>
:policy_network_cnn_linear_matmul_1_readvariableop_resource;
7policy_network_cnn_linear_add_1_readvariableop_resource>
:policy_network_cnn_linear_matmul_2_readvariableop_resource;
7policy_network_cnn_linear_add_2_readvariableop_resource>
:policy_network_cnn_linear_matmul_3_readvariableop_resource;
7policy_network_cnn_linear_add_3_readvariableop_resource>
:policy_network_cnn_linear_matmul_4_readvariableop_resource;
7policy_network_cnn_linear_add_4_readvariableop_resource
identity

identity_1’1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp’3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp’Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp’Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp’,policy_network_cnn/linear/Add/ReadVariableOp’.policy_network_cnn/linear/Add_1/ReadVariableOp’.policy_network_cnn/linear/Add_2/ReadVariableOp’.policy_network_cnn/linear/Add_3/ReadVariableOp’.policy_network_cnn/linear/Add_4/ReadVariableOp’/policy_network_cnn/linear/MatMul/ReadVariableOp’1policy_network_cnn/linear/MatMul_1/ReadVariableOp’1policy_network_cnn/linear/MatMul_2/ReadVariableOp’1policy_network_cnn/linear/MatMul_3/ReadVariableOp’1policy_network_cnn/linear/MatMul_4/ReadVariableOpz
 policy_network_cnn/flatten/ShapeShapeargs_0*
T0*
_output_shapes
:2"
 policy_network_cnn/flatten/Shapeͺ
.policy_network_cnn/flatten/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 20
.policy_network_cnn/flatten/strided_slice/stack?
0policy_network_cnn/flatten/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/flatten/strided_slice/stack_1?
0policy_network_cnn/flatten/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/flatten/strided_slice/stack_2
(policy_network_cnn/flatten/strided_sliceStridedSlice)policy_network_cnn/flatten/Shape:output:07policy_network_cnn/flatten/strided_slice/stack:output:09policy_network_cnn/flatten/strided_slice/stack_1:output:09policy_network_cnn/flatten/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2*
(policy_network_cnn/flatten/strided_slice£
*policy_network_cnn/flatten/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB:β2,
*policy_network_cnn/flatten/concat/values_1
&policy_network_cnn/flatten/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2(
&policy_network_cnn/flatten/concat/axis
!policy_network_cnn/flatten/concatConcatV21policy_network_cnn/flatten/strided_slice:output:03policy_network_cnn/flatten/concat/values_1:output:0/policy_network_cnn/flatten/concat/axis:output:0*
N*
T0*
_output_shapes
:2#
!policy_network_cnn/flatten/concatΊ
"policy_network_cnn/flatten/ReshapeReshapeargs_0*policy_network_cnn/flatten/concat:output:0*
T0*(
_output_shapes
:?????????β2$
"policy_network_cnn/flatten/Reshape
$policy_network_cnn/concat/concat_dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2&
$policy_network_cnn/concat/concat_dim°
 policy_network_cnn/concat/concatIdentity+policy_network_cnn/flatten/Reshape:output:0*
T0*(
_output_shapes
:?????????β2"
 policy_network_cnn/concat/concat
policy_network_cnn/ConstConst*
_output_shapes
:*
dtype0*
valueB"Δ     2
policy_network_cnn/Const
"policy_network_cnn/split/split_dimConst*
_output_shapes
: *
dtype0*
value	B :2$
"policy_network_cnn/split/split_dim
policy_network_cnn/splitSplitV)policy_network_cnn/concat/concat:output:0!policy_network_cnn/Const:output:0+policy_network_cnn/split/split_dim:output:0*
T0*

Tlen0*;
_output_shapes)
':?????????Δ:?????????*
	num_split2
policy_network_cnn/split
 policy_network_cnn/reshape/ShapeShape!policy_network_cnn/split:output:0*
T0*
_output_shapes
:2"
 policy_network_cnn/reshape/Shapeͺ
.policy_network_cnn/reshape/strided_slice/stackConst*
_output_shapes
:*
dtype0*
valueB: 20
.policy_network_cnn/reshape/strided_slice/stack?
0policy_network_cnn/reshape/strided_slice/stack_1Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/reshape/strided_slice/stack_1?
0policy_network_cnn/reshape/strided_slice/stack_2Const*
_output_shapes
:*
dtype0*
valueB:22
0policy_network_cnn/reshape/strided_slice/stack_2
(policy_network_cnn/reshape/strided_sliceStridedSlice)policy_network_cnn/reshape/Shape:output:07policy_network_cnn/reshape/strided_slice/stack:output:09policy_network_cnn/reshape/strided_slice/stack_1:output:09policy_network_cnn/reshape/strided_slice/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2*
(policy_network_cnn/reshape/strided_slice©
*policy_network_cnn/reshape/concat/values_1Const*
_output_shapes
:*
dtype0*
valueB"ρ      2,
*policy_network_cnn/reshape/concat/values_1
&policy_network_cnn/reshape/concat/axisConst*
_output_shapes
: *
dtype0*
value	B : 2(
&policy_network_cnn/reshape/concat/axis
!policy_network_cnn/reshape/concatConcatV21policy_network_cnn/reshape/strided_slice:output:03policy_network_cnn/reshape/concat/values_1:output:0/policy_network_cnn/reshape/concat/axis:output:0*
N*
T0*
_output_shapes
:2#
!policy_network_cnn/reshape/concatΩ
"policy_network_cnn/reshape/ReshapeReshape!policy_network_cnn/split:output:0*policy_network_cnn/reshape/concat:output:0*
T0*,
_output_shapes
:?????????ρ2$
"policy_network_cnn/reshape/ReshapeΉ
5policy_network_cnn/conv1_d/convolution/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ύ????????27
5policy_network_cnn/conv1_d/convolution/ExpandDims/dim
1policy_network_cnn/conv1_d/convolution/ExpandDims
ExpandDims+policy_network_cnn/reshape/Reshape:output:0>policy_network_cnn/conv1_d/convolution/ExpandDims/dim:output:0*
T0*0
_output_shapes
:?????????ρ23
1policy_network_cnn/conv1_d/convolution/ExpandDims
Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpReadVariableOpKpolicy_network_cnn_conv1_d_convolution_expanddims_1_readvariableop_resource*"
_output_shapes
: *
dtype02D
Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp΄
7policy_network_cnn/conv1_d/convolution/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 29
7policy_network_cnn/conv1_d/convolution/ExpandDims_1/dim·
3policy_network_cnn/conv1_d/convolution/ExpandDims_1
ExpandDimsJpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp:value:0@policy_network_cnn/conv1_d/convolution/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
: 25
3policy_network_cnn/conv1_d/convolution/ExpandDims_1·
&policy_network_cnn/conv1_d/convolutionConv2D:policy_network_cnn/conv1_d/convolution/ExpandDims:output:0<policy_network_cnn/conv1_d/convolution/ExpandDims_1:output:0*
T0*0
_output_shapes
:?????????ρ *
paddingSAME*
strides
2(
&policy_network_cnn/conv1_d/convolutionσ
.policy_network_cnn/conv1_d/convolution/SqueezeSqueeze/policy_network_cnn/conv1_d/convolution:output:0*
T0*,
_output_shapes
:?????????ρ *
squeeze_dims

ύ????????20
.policy_network_cnn/conv1_d/convolution/Squeezeέ
1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOpReadVariableOp:policy_network_cnn_conv1_d_biasadd_readvariableop_resource*
_output_shapes
: *
dtype023
1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOpώ
"policy_network_cnn/conv1_d/BiasAddBiasAdd7policy_network_cnn/conv1_d/convolution/Squeeze:output:09policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp:value:0*
T0*,
_output_shapes
:?????????ρ 2$
"policy_network_cnn/conv1_d/BiasAdd΄
"policy_network_cnn/sequential/ReluRelu+policy_network_cnn/conv1_d/BiasAdd:output:0*
T0*,
_output_shapes
:?????????ρ 2$
"policy_network_cnn/sequential/Relu½
7policy_network_cnn/conv1_d/convolution_1/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
ύ????????29
7policy_network_cnn/conv1_d/convolution_1/ExpandDims/dim§
3policy_network_cnn/conv1_d/convolution_1/ExpandDims
ExpandDims0policy_network_cnn/sequential/Relu:activations:0@policy_network_cnn/conv1_d/convolution_1/ExpandDims/dim:output:0*
T0*0
_output_shapes
:?????????ρ 25
3policy_network_cnn/conv1_d/convolution_1/ExpandDims
Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpReadVariableOpMpolicy_network_cnn_conv1_d_convolution_1_expanddims_1_readvariableop_resource*"
_output_shapes
:  *
dtype02F
Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpΈ
9policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2;
9policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/dimΏ
5policy_network_cnn/conv1_d/convolution_1/ExpandDims_1
ExpandDimsLpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp:value:0Bpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:  27
5policy_network_cnn/conv1_d/convolution_1/ExpandDims_1Ώ
(policy_network_cnn/conv1_d/convolution_1Conv2D<policy_network_cnn/conv1_d/convolution_1/ExpandDims:output:0>policy_network_cnn/conv1_d/convolution_1/ExpandDims_1:output:0*
T0*0
_output_shapes
:?????????ρ *
paddingSAME*
strides
2*
(policy_network_cnn/conv1_d/convolution_1ω
0policy_network_cnn/conv1_d/convolution_1/SqueezeSqueeze1policy_network_cnn/conv1_d/convolution_1:output:0*
T0*,
_output_shapes
:?????????ρ *
squeeze_dims

ύ????????22
0policy_network_cnn/conv1_d/convolution_1/Squeezeγ
3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOpReadVariableOp<policy_network_cnn_conv1_d_biasadd_1_readvariableop_resource*
_output_shapes
: *
dtype025
3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp
$policy_network_cnn/conv1_d/BiasAdd_1BiasAdd9policy_network_cnn/conv1_d/convolution_1/Squeeze:output:0;policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp:value:0*
T0*,
_output_shapes
:?????????ρ 2&
$policy_network_cnn/conv1_d/BiasAdd_1Ί
$policy_network_cnn/sequential/Relu_1Relu-policy_network_cnn/conv1_d/BiasAdd_1:output:0*
T0*,
_output_shapes
:?????????ρ 2&
$policy_network_cnn/sequential/Relu_1ͺ
"policy_network_cnn/flatten/Shape_1Shape2policy_network_cnn/sequential/Relu_1:activations:0*
T0*
_output_shapes
:2$
"policy_network_cnn/flatten/Shape_1?
0policy_network_cnn/flatten/strided_slice_1/stackConst*
_output_shapes
:*
dtype0*
valueB: 22
0policy_network_cnn/flatten/strided_slice_1/stack²
2policy_network_cnn/flatten/strided_slice_1/stack_1Const*
_output_shapes
:*
dtype0*
valueB:24
2policy_network_cnn/flatten/strided_slice_1/stack_1²
2policy_network_cnn/flatten/strided_slice_1/stack_2Const*
_output_shapes
:*
dtype0*
valueB:24
2policy_network_cnn/flatten/strided_slice_1/stack_2
*policy_network_cnn/flatten/strided_slice_1StridedSlice+policy_network_cnn/flatten/Shape_1:output:09policy_network_cnn/flatten/strided_slice_1/stack:output:0;policy_network_cnn/flatten/strided_slice_1/stack_1:output:0;policy_network_cnn/flatten/strided_slice_1/stack_2:output:0*
Index0*
T0*
_output_shapes
:*

begin_mask2,
*policy_network_cnn/flatten/strided_slice_1§
,policy_network_cnn/flatten/concat_1/values_1Const*
_output_shapes
:*
dtype0*
valueB: <2.
,policy_network_cnn/flatten/concat_1/values_1
(policy_network_cnn/flatten/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B : 2*
(policy_network_cnn/flatten/concat_1/axis£
#policy_network_cnn/flatten/concat_1ConcatV23policy_network_cnn/flatten/strided_slice_1:output:05policy_network_cnn/flatten/concat_1/values_1:output:01policy_network_cnn/flatten/concat_1/axis:output:0*
N*
T0*
_output_shapes
:2%
#policy_network_cnn/flatten/concat_1μ
$policy_network_cnn/flatten/Reshape_1Reshape2policy_network_cnn/sequential/Relu_1:activations:0,policy_network_cnn/flatten/concat_1:output:0*
T0*(
_output_shapes
:????????? <2&
$policy_network_cnn/flatten/Reshape_1
 policy_network_cnn/concat_1/axisConst*
_output_shapes
: *
dtype0*
value	B :2"
 policy_network_cnn/concat_1/axis?
policy_network_cnn/concat_1ConcatV2!policy_network_cnn/split:output:1-policy_network_cnn/flatten/Reshape_1:output:0)policy_network_cnn/concat_1/axis:output:0*
N*
T0*(
_output_shapes
:?????????Ύ<2
policy_network_cnn/concat_1έ
/policy_network_cnn/linear/MatMul/ReadVariableOpReadVariableOp8policy_network_cnn_linear_matmul_readvariableop_resource* 
_output_shapes
:
Ύ<*
dtype021
/policy_network_cnn/linear/MatMul/ReadVariableOpΰ
 policy_network_cnn/linear/MatMulMatMul$policy_network_cnn/concat_1:output:07policy_network_cnn/linear/MatMul/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2"
 policy_network_cnn/linear/MatMulΟ
,policy_network_cnn/linear/Add/ReadVariableOpReadVariableOp5policy_network_cnn_linear_add_readvariableop_resource*
_output_shapes	
:*
dtype02.
,policy_network_cnn/linear/Add/ReadVariableOpΪ
policy_network_cnn/linear/AddAdd*policy_network_cnn/linear/MatMul:product:04policy_network_cnn/linear/Add/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2
policy_network_cnn/linear/Add£
!policy_network_cnn/sequential/EluElu!policy_network_cnn/linear/Add:z:0*
T0*(
_output_shapes
:?????????2#
!policy_network_cnn/sequential/Eluγ
1policy_network_cnn/linear/MatMul_1/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_1_readvariableop_resource* 
_output_shapes
:
*
dtype023
1policy_network_cnn/linear/MatMul_1/ReadVariableOpρ
"policy_network_cnn/linear/MatMul_1MatMul/policy_network_cnn/sequential/Elu:activations:09policy_network_cnn/linear/MatMul_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_1Υ
.policy_network_cnn/linear/Add_1/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_1_readvariableop_resource*
_output_shapes	
:*
dtype020
.policy_network_cnn/linear/Add_1/ReadVariableOpβ
policy_network_cnn/linear/Add_1Add,policy_network_cnn/linear/MatMul_1:product:06policy_network_cnn/linear/Add_1/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_1©
#policy_network_cnn/sequential/Elu_1Elu#policy_network_cnn/linear/Add_1:z:0*
T0*(
_output_shapes
:?????????2%
#policy_network_cnn/sequential/Elu_1γ
1policy_network_cnn/linear/MatMul_2/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_2_readvariableop_resource* 
_output_shapes
:
*
dtype023
1policy_network_cnn/linear/MatMul_2/ReadVariableOpσ
"policy_network_cnn/linear/MatMul_2MatMul1policy_network_cnn/sequential/Elu_1:activations:09policy_network_cnn/linear/MatMul_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_2Υ
.policy_network_cnn/linear/Add_2/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_2_readvariableop_resource*
_output_shapes	
:*
dtype020
.policy_network_cnn/linear/Add_2/ReadVariableOpβ
policy_network_cnn/linear/Add_2Add,policy_network_cnn/linear/MatMul_2:product:06policy_network_cnn/linear/Add_2/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_2©
#policy_network_cnn/sequential/Elu_2Elu#policy_network_cnn/linear/Add_2:z:0*
T0*(
_output_shapes
:?????????2%
#policy_network_cnn/sequential/Elu_2γ
1policy_network_cnn/linear/MatMul_3/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_3_readvariableop_resource* 
_output_shapes
:
*
dtype023
1policy_network_cnn/linear/MatMul_3/ReadVariableOpσ
"policy_network_cnn/linear/MatMul_3MatMul1policy_network_cnn/sequential/Elu_2:activations:09policy_network_cnn/linear/MatMul_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_3Υ
.policy_network_cnn/linear/Add_3/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_3_readvariableop_resource*
_output_shapes	
:*
dtype020
.policy_network_cnn/linear/Add_3/ReadVariableOpβ
policy_network_cnn/linear/Add_3Add,policy_network_cnn/linear/MatMul_3:product:06policy_network_cnn/linear/Add_3/ReadVariableOp:value:0*
T0*(
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_3©
#policy_network_cnn/sequential/Elu_3Elu#policy_network_cnn/linear/Add_3:z:0*
T0*(
_output_shapes
:?????????2%
#policy_network_cnn/sequential/Elu_3β
1policy_network_cnn/linear/MatMul_4/ReadVariableOpReadVariableOp:policy_network_cnn_linear_matmul_4_readvariableop_resource*
_output_shapes
:	*
dtype023
1policy_network_cnn/linear/MatMul_4/ReadVariableOpς
"policy_network_cnn/linear/MatMul_4MatMul1policy_network_cnn/sequential/Elu_3:activations:09policy_network_cnn/linear/MatMul_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2$
"policy_network_cnn/linear/MatMul_4Τ
.policy_network_cnn/linear/Add_4/ReadVariableOpReadVariableOp7policy_network_cnn_linear_add_4_readvariableop_resource*
_output_shapes
:*
dtype020
.policy_network_cnn/linear/Add_4/ReadVariableOpα
policy_network_cnn/linear/Add_4Add,policy_network_cnn/linear/MatMul_4:product:06policy_network_cnn/linear/Add_4/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2!
policy_network_cnn/linear/Add_4«
$policy_network_cnn/tanh_to_spec/TanhTanh#policy_network_cnn/linear/Add_4:z:0*
T0*'
_output_shapes
:?????????2&
$policy_network_cnn/tanh_to_spec/Tanh
%policy_network_cnn/tanh_to_spec/add/yConst*
_output_shapes
: *
dtype0*
valueB
 *  ?2'
%policy_network_cnn/tanh_to_spec/add/yί
#policy_network_cnn/tanh_to_spec/addAddV2(policy_network_cnn/tanh_to_spec/Tanh:y:0.policy_network_cnn/tanh_to_spec/add/y:output:0*
T0*'
_output_shapes
:?????????2%
#policy_network_cnn/tanh_to_spec/add
%policy_network_cnn/tanh_to_spec/mul/xConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2'
%policy_network_cnn/tanh_to_spec/mul/xά
#policy_network_cnn/tanh_to_spec/mulMul.policy_network_cnn/tanh_to_spec/mul/x:output:0'policy_network_cnn/tanh_to_spec/add:z:0*
T0*'
_output_shapes
:?????????2%
#policy_network_cnn/tanh_to_spec/mul£
'policy_network_cnn/tanh_to_spec/mul_1/yConst*
_output_shapes
:*
dtype0*
valueB"  ?   @2)
'policy_network_cnn/tanh_to_spec/mul_1/yβ
%policy_network_cnn/tanh_to_spec/mul_1Mul'policy_network_cnn/tanh_to_spec/mul:z:00policy_network_cnn/tanh_to_spec/mul_1/y:output:0*
T0*'
_output_shapes
:?????????2'
%policy_network_cnn/tanh_to_spec/mul_1£
'policy_network_cnn/tanh_to_spec/add_1/yConst*
_output_shapes
:*
dtype0*
valueB"      Ώ2)
'policy_network_cnn/tanh_to_spec/add_1/yζ
%policy_network_cnn/tanh_to_spec/add_1AddV2)policy_network_cnn/tanh_to_spec/mul_1:z:00policy_network_cnn/tanh_to_spec/add_1/y:output:0*
T0*'
_output_shapes
:?????????2'
%policy_network_cnn/tanh_to_spec/add_1ν
IdentityIdentity-policy_network_cnn/flatten/Reshape_1:output:02^policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp4^policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOpC^policy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpE^policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp-^policy_network_cnn/linear/Add/ReadVariableOp/^policy_network_cnn/linear/Add_1/ReadVariableOp/^policy_network_cnn/linear/Add_2/ReadVariableOp/^policy_network_cnn/linear/Add_3/ReadVariableOp/^policy_network_cnn/linear/Add_4/ReadVariableOp0^policy_network_cnn/linear/MatMul/ReadVariableOp2^policy_network_cnn/linear/MatMul_1/ReadVariableOp2^policy_network_cnn/linear/MatMul_2/ReadVariableOp2^policy_network_cnn/linear/MatMul_3/ReadVariableOp2^policy_network_cnn/linear/MatMul_4/ReadVariableOp*
T0*(
_output_shapes
:????????? <2

Identityμ

Identity_1Identity)policy_network_cnn/tanh_to_spec/add_1:z:02^policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp4^policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOpC^policy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpE^policy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp-^policy_network_cnn/linear/Add/ReadVariableOp/^policy_network_cnn/linear/Add_1/ReadVariableOp/^policy_network_cnn/linear/Add_2/ReadVariableOp/^policy_network_cnn/linear/Add_3/ReadVariableOp/^policy_network_cnn/linear/Add_4/ReadVariableOp0^policy_network_cnn/linear/MatMul/ReadVariableOp2^policy_network_cnn/linear/MatMul_1/ReadVariableOp2^policy_network_cnn/linear/MatMul_2/ReadVariableOp2^policy_network_cnn/linear/MatMul_3/ReadVariableOp2^policy_network_cnn/linear/MatMul_4/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity_1"
identityIdentity:output:0"!

identity_1Identity_1:output:0*_
_input_shapesN
L:?????????β::::::::::::::2f
1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp1policy_network_cnn/conv1_d/BiasAdd/ReadVariableOp2j
3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp3policy_network_cnn/conv1_d/BiasAdd_1/ReadVariableOp2
Bpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOpBpolicy_network_cnn/conv1_d/convolution/ExpandDims_1/ReadVariableOp2
Dpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOpDpolicy_network_cnn/conv1_d/convolution_1/ExpandDims_1/ReadVariableOp2\
,policy_network_cnn/linear/Add/ReadVariableOp,policy_network_cnn/linear/Add/ReadVariableOp2`
.policy_network_cnn/linear/Add_1/ReadVariableOp.policy_network_cnn/linear/Add_1/ReadVariableOp2`
.policy_network_cnn/linear/Add_2/ReadVariableOp.policy_network_cnn/linear/Add_2/ReadVariableOp2`
.policy_network_cnn/linear/Add_3/ReadVariableOp.policy_network_cnn/linear/Add_3/ReadVariableOp2`
.policy_network_cnn/linear/Add_4/ReadVariableOp.policy_network_cnn/linear/Add_4/ReadVariableOp2b
/policy_network_cnn/linear/MatMul/ReadVariableOp/policy_network_cnn/linear/MatMul/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_1/ReadVariableOp1policy_network_cnn/linear/MatMul_1/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_2/ReadVariableOp1policy_network_cnn/linear/MatMul_2/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_3/ReadVariableOp1policy_network_cnn/linear/MatMul_3/ReadVariableOp2f
1policy_network_cnn/linear/MatMul_4/ReadVariableOp1policy_network_cnn/linear/MatMul_4/ReadVariableOp:P L
(
_output_shapes
:?????????β
 
_user_specified_nameargs_0
γ)
Ν
 __inference__traced_save_2911288
file_prefix;
7savev2_policy_network_cnn_conv1_d_b_read_readvariableop;
7savev2_policy_network_cnn_conv1_d_w_read_readvariableop=
9savev2_policy_network_cnn_conv1_d_b_1_read_readvariableop=
9savev2_policy_network_cnn_conv1_d_w_1_read_readvariableop:
6savev2_policy_network_cnn_linear_b_read_readvariableop:
6savev2_policy_network_cnn_linear_w_read_readvariableop<
8savev2_policy_network_cnn_linear_b_1_read_readvariableop<
8savev2_policy_network_cnn_linear_w_1_read_readvariableop<
8savev2_policy_network_cnn_linear_b_2_read_readvariableop<
8savev2_policy_network_cnn_linear_w_2_read_readvariableop<
8savev2_policy_network_cnn_linear_b_3_read_readvariableop<
8savev2_policy_network_cnn_linear_w_3_read_readvariableop<
8savev2_policy_network_cnn_linear_b_4_read_readvariableop<
8savev2_policy_network_cnn_linear_w_4_read_readvariableop
savev2_const

identity_1’MergeV2Checkpoints
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
Constl
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part2	
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
ShardedFilenameγ
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*υ
valueλBθB'_variables/0/.ATTRIBUTES/VARIABLE_VALUEB'_variables/1/.ATTRIBUTES/VARIABLE_VALUEB'_variables/2/.ATTRIBUTES/VARIABLE_VALUEB'_variables/3/.ATTRIBUTES/VARIABLE_VALUEB'_variables/4/.ATTRIBUTES/VARIABLE_VALUEB'_variables/5/.ATTRIBUTES/VARIABLE_VALUEB'_variables/6/.ATTRIBUTES/VARIABLE_VALUEB'_variables/7/.ATTRIBUTES/VARIABLE_VALUEB'_variables/8/.ATTRIBUTES/VARIABLE_VALUEB'_variables/9/.ATTRIBUTES/VARIABLE_VALUEB(_variables/10/.ATTRIBUTES/VARIABLE_VALUEB(_variables/11/.ATTRIBUTES/VARIABLE_VALUEB(_variables/12/.ATTRIBUTES/VARIABLE_VALUEB(_variables/13/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
SaveV2/tensor_names¦
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*1
value(B&B B B B B B B B B B B B B B B 2
SaveV2/shape_and_slicesπ
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:07savev2_policy_network_cnn_conv1_d_b_read_readvariableop7savev2_policy_network_cnn_conv1_d_w_read_readvariableop9savev2_policy_network_cnn_conv1_d_b_1_read_readvariableop9savev2_policy_network_cnn_conv1_d_w_1_read_readvariableop6savev2_policy_network_cnn_linear_b_read_readvariableop6savev2_policy_network_cnn_linear_w_read_readvariableop8savev2_policy_network_cnn_linear_b_1_read_readvariableop8savev2_policy_network_cnn_linear_w_1_read_readvariableop8savev2_policy_network_cnn_linear_b_2_read_readvariableop8savev2_policy_network_cnn_linear_w_2_read_readvariableop8savev2_policy_network_cnn_linear_b_3_read_readvariableop8savev2_policy_network_cnn_linear_w_3_read_readvariableop8savev2_policy_network_cnn_linear_b_4_read_readvariableop8savev2_policy_network_cnn_linear_w_4_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 *
dtypes
22
SaveV2Ί
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:2(
&MergeV2Checkpoints/checkpoint_prefixes‘
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
Ύ<::
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
Ύ<:!
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
ξ>
δ
#__inference__traced_restore_2911340
file_prefix1
-assignvariableop_policy_network_cnn_conv1_d_b3
/assignvariableop_1_policy_network_cnn_conv1_d_w5
1assignvariableop_2_policy_network_cnn_conv1_d_b_15
1assignvariableop_3_policy_network_cnn_conv1_d_w_12
.assignvariableop_4_policy_network_cnn_linear_b2
.assignvariableop_5_policy_network_cnn_linear_w4
0assignvariableop_6_policy_network_cnn_linear_b_14
0assignvariableop_7_policy_network_cnn_linear_w_14
0assignvariableop_8_policy_network_cnn_linear_b_24
0assignvariableop_9_policy_network_cnn_linear_w_25
1assignvariableop_10_policy_network_cnn_linear_b_35
1assignvariableop_11_policy_network_cnn_linear_w_35
1assignvariableop_12_policy_network_cnn_linear_b_45
1assignvariableop_13_policy_network_cnn_linear_w_4
identity_15’AssignVariableOp’AssignVariableOp_1’AssignVariableOp_10’AssignVariableOp_11’AssignVariableOp_12’AssignVariableOp_13’AssignVariableOp_2’AssignVariableOp_3’AssignVariableOp_4’AssignVariableOp_5’AssignVariableOp_6’AssignVariableOp_7’AssignVariableOp_8’AssignVariableOp_9ι
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*υ
valueλBθB'_variables/0/.ATTRIBUTES/VARIABLE_VALUEB'_variables/1/.ATTRIBUTES/VARIABLE_VALUEB'_variables/2/.ATTRIBUTES/VARIABLE_VALUEB'_variables/3/.ATTRIBUTES/VARIABLE_VALUEB'_variables/4/.ATTRIBUTES/VARIABLE_VALUEB'_variables/5/.ATTRIBUTES/VARIABLE_VALUEB'_variables/6/.ATTRIBUTES/VARIABLE_VALUEB'_variables/7/.ATTRIBUTES/VARIABLE_VALUEB'_variables/8/.ATTRIBUTES/VARIABLE_VALUEB'_variables/9/.ATTRIBUTES/VARIABLE_VALUEB(_variables/10/.ATTRIBUTES/VARIABLE_VALUEB(_variables/11/.ATTRIBUTES/VARIABLE_VALUEB(_variables/12/.ATTRIBUTES/VARIABLE_VALUEB(_variables/13/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
RestoreV2/tensor_names¬
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*1
value(B&B B B B B B B B B B B B B B B 2
RestoreV2/shape_and_slicesφ
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

Identity¬
AssignVariableOpAssignVariableOp-assignvariableop_policy_network_cnn_conv1_d_bIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1΄
AssignVariableOp_1AssignVariableOp/assignvariableop_1_policy_network_cnn_conv1_d_wIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2Ά
AssignVariableOp_2AssignVariableOp1assignvariableop_2_policy_network_cnn_conv1_d_b_1Identity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3Ά
AssignVariableOp_3AssignVariableOp1assignvariableop_3_policy_network_cnn_conv1_d_w_1Identity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4³
AssignVariableOp_4AssignVariableOp.assignvariableop_4_policy_network_cnn_linear_bIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5³
AssignVariableOp_5AssignVariableOp.assignvariableop_5_policy_network_cnn_linear_wIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_5k

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0*
_output_shapes
:2

Identity_6΅
AssignVariableOp_6AssignVariableOp0assignvariableop_6_policy_network_cnn_linear_b_1Identity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_6k

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:2

Identity_7΅
AssignVariableOp_7AssignVariableOp0assignvariableop_7_policy_network_cnn_linear_w_1Identity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_7k

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:2

Identity_8΅
AssignVariableOp_8AssignVariableOp0assignvariableop_8_policy_network_cnn_linear_b_2Identity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_8k

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:2

Identity_9΅
AssignVariableOp_9AssignVariableOp0assignvariableop_9_policy_network_cnn_linear_w_2Identity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_9n
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:2
Identity_10Ή
AssignVariableOp_10AssignVariableOp1assignvariableop_10_policy_network_cnn_linear_b_3Identity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_10n
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:2
Identity_11Ή
AssignVariableOp_11AssignVariableOp1assignvariableop_11_policy_network_cnn_linear_w_3Identity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_11n
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:2
Identity_12Ή
AssignVariableOp_12AssignVariableOp1assignvariableop_12_policy_network_cnn_linear_b_4Identity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_12n
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:2
Identity_13Ή
AssignVariableOp_13AssignVariableOp1assignvariableop_13_policy_network_cnn_linear_w_4Identity_13:output:0"/device:CPU:0*
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
_user_specified_namefile_prefix"±J
saver_filename:0StatefulPartitionedCall:0StatefulPartitionedCall_18"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp:
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
*:( 2policy_network_cnn/conv1_d/b
2:0 2policy_network_cnn/conv1_d/w
*:( 2policy_network_cnn/conv1_d/b
2:0  2policy_network_cnn/conv1_d/w
*:(2policy_network_cnn/linear/b
/:-
Ύ<2policy_network_cnn/linear/w
*:(2policy_network_cnn/linear/b
/:-
2policy_network_cnn/linear/w
*:(2policy_network_cnn/linear/b
/:-
2policy_network_cnn/linear/w
*:(2policy_network_cnn/linear/b
/:-
2policy_network_cnn/linear/w
):'2policy_network_cnn/linear/b
.:,	2policy_network_cnn/linear/w
Ζ2Γ
__inference___call___2911118’
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
annotationsͺ *
 
Β2Ώ
"__inference_wrapped_module_2911225
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
annotationsͺ *
 £
__inference___call___2911118	
0’-
&’#
!
args_0?????????β
ͺ ">’;

0????????? <

1?????????©
"__inference_wrapped_module_2911225	
0’-
&’#
!
args_0?????????β
ͺ ">’;

0????????? <

1?????????