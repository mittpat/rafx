                       �""D@"���Ȟ
3��5�]      �[      struct HistogramResult
{
    float average_luminosity_interpolated;
    float average_luminosity_this_frame;
    float average_luminosity_last_frame;
    float min_luminosity_interpolated;
    float min_luminosity_this_frame;
    float min_luminosity_last_frame;
    float max_luminosity_interpolated;
    float max_luminosity_this_frame;
    float max_luminosity_last_frame;
    float low_luminosity_interpolated;
    float low_luminosity_this_frame;
    float low_luminosity_last_frame;
    float high_luminosity_interpolated;
    float high_luminosity_this_frame;
    float high_luminosity_last_frame;
    float average_bin_include_zero;
    float average_bin_non_zero;
    uint min_bin;
    uint max_bin;
    uint low_bin;
    uint high_bin;
};

static const uint3 gl_WorkGroupSize = uint3(256u, 1u, 1u);

RWByteAddressBuffer histogram_data : register(u0, space0);
cbuffer AverageHistogramConfig : register(b1, space0)
{
    uint config_pixel_count : packoffset(c0);
    float config_min_log_luma : packoffset(c0.y);
    float config_log_luma_range : packoffset(c0.z);
    float config_dt : packoffset(c0.w);
    float config_low_percentile : packoffset(c1);
    float config_high_percentile : packoffset(c1.y);
    float config_low_adjust_speed : packoffset(c1.z);
    float config_high_adjust_speed : packoffset(c1.w);
    uint config_write_debug_output : packoffset(c2);
};

RWByteAddressBuffer histogram_result : register(u2, space0);
RWByteAddressBuffer debug_output : register(u3, space0);

static uint gl_LocalInvocationIndex;
struct SPIRV_Cross_Input
{
    uint gl_LocalInvocationIndex : SV_GroupIndex;
};

groupshared float HistogramShared[256];

float bin_to_luminosity(float bin, float min_log_luma, float log_luma_range)
{
    return exp2(((bin / 255.0f) * log_luma_range) + min_log_luma);
}

void comp_main()
{
    float count_for_this_bin = float(histogram_data.Load(gl_LocalInvocationIndex * 4 + 0));
    HistogramShared[gl_LocalInvocationIndex] = count_for_this_bin * float(gl_LocalInvocationIndex + 1u);
    GroupMemoryBarrierWithGroupSync();
    uint histogram_sample_index = 128u;
    for (;;)
    {
        if (histogram_sample_index > 0u)
        {
            if (gl_LocalInvocationIndex < histogram_sample_index)
            {
                HistogramShared[gl_LocalInvocationIndex] += HistogramShared[gl_LocalInvocationIndex + histogram_sample_index];
            }
            GroupMemoryBarrierWithGroupSync();
            histogram_sample_index = histogram_sample_index >> uint(1);
            continue;
        }
        else
        {
            break;
        }
    }
    if (gl_LocalInvocationIndex == 0u)
    {
        uint zero_pixel_count = uint(count_for_this_bin);
        uint max_bin = 0u;
        uint min_bin = 255u;
        uint pixels_seen = 0u;
        uint high_bin = 0u;
        uint high_pixel_thresh = uint((1.0f - config_high_percentile) * float(config_pixel_count - zero_pixel_count));
        uint low_bin = 0u;
        uint low_pixel_thresh = uint((1.0f - config_low_percentile) * float(config_pixel_count - zero_pixel_count));
        int i = 255;
        for (;;)
        {
            if (i >= 0)
            {
                uint data = histogram_data.Load(i * 4 + 0);
                if (data > 0u)
                {
                    max_bin = max(max_bin, uint(i));
                    min_bin = uint(i);
                }
                pixels_seen += data;
                if (pixels_seen > high_pixel_thresh)
                {
                    high_bin = max(high_bin, uint(i));
                }
                if (pixels_seen > low_pixel_thresh)
                {
                    low_bin = max(low_bin, uint(i));
                }
                i--;
                continue;
            }
            else
            {
                break;
            }
        }
        float average_bin_include_zero = (HistogramShared[0] / float(config_pixel_count)) - 1.0f;
        float non_zero_pixel_count = float(config_pixel_count) - float(zero_pixel_count);
        float average_bin_non_zero = (HistogramShared[0] - non_zero_pixel_count) / max(non_zero_pixel_count, 1.0f);
        float average_bin = average_bin_include_zero;
        histogram_result.Store(8, asuint(asfloat(histogram_result.Load(0))));
        histogram_result.Store(20, asuint(asfloat(histogram_result.Load(12))));
        histogram_result.Store(32, asuint(asfloat(histogram_result.Load(24))));
        histogram_result.Store(44, asuint(asfloat(histogram_result.Load(36))));
        histogram_result.Store(56, asuint(asfloat(histogram_result.Load(48))));
        float param = average_bin;
        float param_1 = config_min_log_luma;
        float param_2 = config_log_luma_range;
        histogram_result.Store(4, asuint(bin_to_luminosity(param, param_1, param_2)));
        float param_3 = float(min_bin);
        float param_4 = config_min_log_luma;
        float param_5 = config_log_luma_range;
        histogram_result.Store(16, asuint(bin_to_luminosity(param_3, param_4, param_5)));
        float param_6 = float(max_bin);
        float param_7 = config_min_log_luma;
        float param_8 = config_log_luma_range;
        histogram_result.Store(28, asuint(bin_to_luminosity(param_6, param_7, param_8)));
        float param_9 = float(low_bin);
        float param_10 = config_min_log_luma;
        float param_11 = config_log_luma_range;
        histogram_result.Store(40, asuint(bin_to_luminosity(param_9, param_10, param_11)));
        float param_12 = float(high_bin);
        float param_13 = config_min_log_luma;
        float param_14 = config_log_luma_range;
        histogram_result.Store(52, asuint(bin_to_luminosity(param_12, param_13, param_14)));
        float interp_high = clamp(1.0f - exp((-config_high_adjust_speed) * config_dt), 0.0f, 1.0f);
        float interp_low = clamp(1.0f - exp((-config_low_adjust_speed) * config_dt), 0.0f, 1.0f);
        histogram_result.Store(0, asuint(lerp(asfloat(histogram_result.Load(0)), asfloat(histogram_result.Load(4)), (asfloat(histogram_result.Load(0)) < asfloat(histogram_result.Load(4))) ? interp_high : interp_low)));
        histogram_result.Store(12, asuint(lerp(asfloat(histogram_result.Load(12)), asfloat(histogram_result.Load(16)), (asfloat(histogram_result.Load(12)) < asfloat(histogram_result.Load(16))) ? interp_high : interp_low)));
        histogram_result.Store(24, asuint(lerp(asfloat(histogram_result.Load(24)), asfloat(histogram_result.Load(28)), (asfloat(histogram_result.Load(24)) < asfloat(histogram_result.Load(28))) ? interp_high : interp_low)));
        histogram_result.Store(36, asuint(lerp(asfloat(histogram_result.Load(36)), asfloat(histogram_result.Load(40)), (asfloat(histogram_result.Load(36)) < asfloat(histogram_result.Load(40))) ? interp_high : interp_low)));
        histogram_result.Store(48, asuint(lerp(asfloat(histogram_result.Load(48)), asfloat(histogram_result.Load(52)), (asfloat(histogram_result.Load(48)) < asfloat(histogram_result.Load(52))) ? interp_high : interp_low)));
        histogram_result.Store(36, asuint(max(asfloat(histogram_result.Load(12)), asfloat(histogram_result.Load(36)))));
        histogram_result.Store(48, asuint(max(asfloat(histogram_result.Load(36)), asfloat(histogram_result.Load(48)))));
        histogram_result.Store(24, asuint(max(asfloat(histogram_result.Load(48)), asfloat(histogram_result.Load(24)))));
        histogram_result.Store(60, asuint(average_bin_include_zero));
        histogram_result.Store(64, asuint(average_bin_non_zero));
        histogram_result.Store(68, min_bin);
        histogram_result.Store(72, max_bin);
        histogram_result.Store(76, low_bin);
        histogram_result.Store(80, high_bin);
        if (config_write_debug_output != 0u)
        {
            HistogramResult _422;
            _422.average_luminosity_interpolated = asfloat(histogram_result.Load(0));
            _422.average_luminosity_this_frame = asfloat(histogram_result.Load(4));
            _422.average_luminosity_last_frame = asfloat(histogram_result.Load(8));
            _422.min_luminosity_interpolated = asfloat(histogram_result.Load(12));
            _422.min_luminosity_this_frame = asfloat(histogram_result.Load(16));
            _422.min_luminosity_last_frame = asfloat(histogram_result.Load(20));
            _422.max_luminosity_interpolated = asfloat(histogram_result.Load(24));
            _422.max_luminosity_this_frame = asfloat(histogram_result.Load(28));
            _422.max_luminosity_last_frame = asfloat(histogram_result.Load(32));
            _422.low_luminosity_interpolated = asfloat(histogram_result.Load(36));
            _422.low_luminosity_this_frame = asfloat(histogram_result.Load(40));
            _422.low_luminosity_last_frame = asfloat(histogram_result.Load(44));
            _422.high_luminosity_interpolated = asfloat(histogram_result.Load(48));
            _422.high_luminosity_this_frame = asfloat(histogram_result.Load(52));
            _422.high_luminosity_last_frame = asfloat(histogram_result.Load(56));
            _422.average_bin_include_zero = asfloat(histogram_result.Load(60));
            _422.average_bin_non_zero = asfloat(histogram_result.Load(64));
            _422.min_bin = histogram_result.Load(68);
            _422.max_bin = histogram_result.Load(72);
            _422.low_bin = histogram_result.Load(76);
            _422.high_bin = histogram_result.Load(80);
            debug_output.Store(0, asuint(_422.average_luminosity_interpolated));
            debug_output.Store(4, asuint(_422.average_luminosity_this_frame));
            debug_output.Store(8, asuint(_422.average_luminosity_last_frame));
            debug_output.Store(12, asuint(_422.min_luminosity_interpolated));
            debug_output.Store(16, asuint(_422.min_luminosity_this_frame));
            debug_output.Store(20, asuint(_422.min_luminosity_last_frame));
            debug_output.Store(24, asuint(_422.max_luminosity_interpolated));
            debug_output.Store(28, asuint(_422.max_luminosity_this_frame));
            debug_output.Store(32, asuint(_422.max_luminosity_last_frame));
            debug_output.Store(36, asuint(_422.low_luminosity_interpolated));
            debug_output.Store(40, asuint(_422.low_luminosity_this_frame));
            debug_output.Store(44, asuint(_422.low_luminosity_last_frame));
            debug_output.Store(48, asuint(_422.high_luminosity_interpolated));
            debug_output.Store(52, asuint(_422.high_luminosity_this_frame));
            debug_output.Store(56, asuint(_422.high_luminosity_last_frame));
            debug_output.Store(60, asuint(_422.average_bin_include_zero));
            debug_output.Store(64, asuint(_422.average_bin_non_zero));
            debug_output.Store(68, _422.min_bin);
            debug_output.Store(72, _422.max_bin);
            debug_output.Store(76, _422.low_bin);
            debug_output.Store(80, _422.high_bin);
            uint _426[256];
            [unroll]
            for (int _0ident = 0; _0ident < 256; _0ident++)
            {
                _426[_0ident] = histogram_data.Load(_0ident * 4 + 0);
            }
            debug_output.Store(84, _426[0]);
            debug_output.Store(88, _426[1]);
            debug_output.Store(92, _426[2]);
            debug_output.Store(96, _426[3]);
            debug_output.Store(100, _426[4]);
            debug_output.Store(104, _426[5]);
            debug_output.Store(108, _426[6]);
            debug_output.Store(112, _426[7]);
            debug_output.Store(116, _426[8]);
            debug_output.Store(120, _426[9]);
            debug_output.Store(124, _426[10]);
            debug_output.Store(128, _426[11]);
            debug_output.Store(132, _426[12]);
            debug_output.Store(136, _426[13]);
            debug_output.Store(140, _426[14]);
            debug_output.Store(144, _426[15]);
            debug_output.Store(148, _426[16]);
            debug_output.Store(152, _426[17]);
            debug_output.Store(156, _426[18]);
            debug_output.Store(160, _426[19]);
            debug_output.Store(164, _426[20]);
            debug_output.Store(168, _426[21]);
            debug_output.Store(172, _426[22]);
            debug_output.Store(176, _426[23]);
            debug_output.Store(180, _426[24]);
            debug_output.Store(184, _426[25]);
            debug_output.Store(188, _426[26]);
            debug_output.Store(192, _426[27]);
            debug_output.Store(196, _426[28]);
            debug_output.Store(200, _426[29]);
            debug_output.Store(204, _426[30]);
            debug_output.Store(208, _426[31]);
            debug_output.Store(212, _426[32]);
            debug_output.Store(216, _426[33]);
            debug_output.Store(220, _426[34]);
            debug_output.Store(224, _426[35]);
            debug_output.Store(228, _426[36]);
            debug_output.Store(232, _426[37]);
            debug_output.Store(236, _426[38]);
            debug_output.Store(240, _426[39]);
            debug_output.Store(244, _426[40]);
            debug_output.Store(248, _426[41]);
            debug_output.Store(252, _426[42]);
            debug_output.Store(256, _426[43]);
            debug_output.Store(260, _426[44]);
            debug_output.Store(264, _426[45]);
            debug_output.Store(268, _426[46]);
            debug_output.Store(272, _426[47]);
            debug_output.Store(276, _426[48]);
            debug_output.Store(280, _426[49]);
            debug_output.Store(284, _426[50]);
            debug_output.Store(288, _426[51]);
            debug_output.Store(292, _426[52]);
            debug_output.Store(296, _426[53]);
            debug_output.Store(300, _426[54]);
            debug_output.Store(304, _426[55]);
            debug_output.Store(308, _426[56]);
            debug_output.Store(312, _426[57]);
            debug_output.Store(316, _426[58]);
            debug_output.Store(320, _426[59]);
            debug_output.Store(324, _426[60]);
            debug_output.Store(328, _426[61]);
            debug_output.Store(332, _426[62]);
            debug_output.Store(336, _426[63]);
            debug_output.Store(340, _426[64]);
            debug_output.Store(344, _426[65]);
            debug_output.Store(348, _426[66]);
            debug_output.Store(352, _426[67]);
            debug_output.Store(356, _426[68]);
            debug_output.Store(360, _426[69]);
            debug_output.Store(364, _426[70]);
            debug_output.Store(368, _426[71]);
            debug_output.Store(372, _426[72]);
            debug_output.Store(376, _426[73]);
            debug_output.Store(380, _426[74]);
            debug_output.Store(384, _426[75]);
            debug_output.Store(388, _426[76]);
            debug_output.Store(392, _426[77]);
            debug_output.Store(396, _426[78]);
            debug_output.Store(400, _426[79]);
            debug_output.Store(404, _426[80]);
            debug_output.Store(408, _426[81]);
            debug_output.Store(412, _426[82]);
            debug_output.Store(416, _426[83]);
            debug_output.Store(420, _426[84]);
            debug_output.Store(424, _426[85]);
            debug_output.Store(428, _426[86]);
            debug_output.Store(432, _426[87]);
            debug_output.Store(436, _426[88]);
            debug_output.Store(440, _426[89]);
            debug_output.Store(444, _426[90]);
            debug_output.Store(448, _426[91]);
            debug_output.Store(452, _426[92]);
            debug_output.Store(456, _426[93]);
            debug_output.Store(460, _426[94]);
            debug_output.Store(464, _426[95]);
            debug_output.Store(468, _426[96]);
            debug_output.Store(472, _426[97]);
            debug_output.Store(476, _426[98]);
            debug_output.Store(480, _426[99]);
            debug_output.Store(484, _426[100]);
            debug_output.Store(488, _426[101]);
            debug_output.Store(492, _426[102]);
            debug_output.Store(496, _426[103]);
            debug_output.Store(500, _426[104]);
            debug_output.Store(504, _426[105]);
            debug_output.Store(508, _426[106]);
            debug_output.Store(512, _426[107]);
            debug_output.Store(516, _426[108]);
            debug_output.Store(520, _426[109]);
            debug_output.Store(524, _426[110]);
            debug_output.Store(528, _426[111]);
            debug_output.Store(532, _426[112]);
            debug_output.Store(536, _426[113]);
            debug_output.Store(540, _426[114]);
            debug_output.Store(544, _426[115]);
            debug_output.Store(548, _426[116]);
            debug_output.Store(552, _426[117]);
            debug_output.Store(556, _426[118]);
            debug_output.Store(560, _426[119]);
            debug_output.Store(564, _426[120]);
            debug_output.Store(568, _426[121]);
            debug_output.Store(572, _426[122]);
            debug_output.Store(576, _426[123]);
            debug_output.Store(580, _426[124]);
            debug_output.Store(584, _426[125]);
            debug_output.Store(588, _426[126]);
            debug_output.Store(592, _426[127]);
            debug_output.Store(596, _426[128]);
            debug_output.Store(600, _426[129]);
            debug_output.Store(604, _426[130]);
            debug_output.Store(608, _426[131]);
            debug_output.Store(612, _426[132]);
            debug_output.Store(616, _426[133]);
            debug_output.Store(620, _426[134]);
            debug_output.Store(624, _426[135]);
            debug_output.Store(628, _426[136]);
            debug_output.Store(632, _426[137]);
            debug_output.Store(636, _426[138]);
            debug_output.Store(640, _426[139]);
            debug_output.Store(644, _426[140]);
            debug_output.Store(648, _426[141]);
            debug_output.Store(652, _426[142]);
            debug_output.Store(656, _426[143]);
            debug_output.Store(660, _426[144]);
            debug_output.Store(664, _426[145]);
            debug_output.Store(668, _426[146]);
            debug_output.Store(672, _426[147]);
            debug_output.Store(676, _426[148]);
            debug_output.Store(680, _426[149]);
            debug_output.Store(684, _426[150]);
            debug_output.Store(688, _426[151]);
            debug_output.Store(692, _426[152]);
            debug_output.Store(696, _426[153]);
            debug_output.Store(700, _426[154]);
            debug_output.Store(704, _426[155]);
            debug_output.Store(708, _426[156]);
            debug_output.Store(712, _426[157]);
            debug_output.Store(716, _426[158]);
            debug_output.Store(720, _426[159]);
            debug_output.Store(724, _426[160]);
            debug_output.Store(728, _426[161]);
            debug_output.Store(732, _426[162]);
            debug_output.Store(736, _426[163]);
            debug_output.Store(740, _426[164]);
            debug_output.Store(744, _426[165]);
            debug_output.Store(748, _426[166]);
            debug_output.Store(752, _426[167]);
            debug_output.Store(756, _426[168]);
            debug_output.Store(760, _426[169]);
            debug_output.Store(764, _426[170]);
            debug_output.Store(768, _426[171]);
            debug_output.Store(772, _426[172]);
            debug_output.Store(776, _426[173]);
            debug_output.Store(780, _426[174]);
            debug_output.Store(784, _426[175]);
            debug_output.Store(788, _426[176]);
            debug_output.Store(792, _426[177]);
            debug_output.Store(796, _426[178]);
            debug_output.Store(800, _426[179]);
            debug_output.Store(804, _426[180]);
            debug_output.Store(808, _426[181]);
            debug_output.Store(812, _426[182]);
            debug_output.Store(816, _426[183]);
            debug_output.Store(820, _426[184]);
            debug_output.Store(824, _426[185]);
            debug_output.Store(828, _426[186]);
            debug_output.Store(832, _426[187]);
            debug_output.Store(836, _426[188]);
            debug_output.Store(840, _426[189]);
            debug_output.Store(844, _426[190]);
            debug_output.Store(848, _426[191]);
            debug_output.Store(852, _426[192]);
            debug_output.Store(856, _426[193]);
            debug_output.Store(860, _426[194]);
            debug_output.Store(864, _426[195]);
            debug_output.Store(868, _426[196]);
            debug_output.Store(872, _426[197]);
            debug_output.Store(876, _426[198]);
            debug_output.Store(880, _426[199]);
            debug_output.Store(884, _426[200]);
            debug_output.Store(888, _426[201]);
            debug_output.Store(892, _426[202]);
            debug_output.Store(896, _426[203]);
            debug_output.Store(900, _426[204]);
            debug_output.Store(904, _426[205]);
            debug_output.Store(908, _426[206]);
            debug_output.Store(912, _426[207]);
            debug_output.Store(916, _426[208]);
            debug_output.Store(920, _426[209]);
            debug_output.Store(924, _426[210]);
            debug_output.Store(928, _426[211]);
            debug_output.Store(932, _426[212]);
            debug_output.Store(936, _426[213]);
            debug_output.Store(940, _426[214]);
            debug_output.Store(944, _426[215]);
            debug_output.Store(948, _426[216]);
            debug_output.Store(952, _426[217]);
            debug_output.Store(956, _426[218]);
            debug_output.Store(960, _426[219]);
            debug_output.Store(964, _426[220]);
            debug_output.Store(968, _426[221]);
            debug_output.Store(972, _426[222]);
            debug_output.Store(976, _426[223]);
            debug_output.Store(980, _426[224]);
            debug_output.Store(984, _426[225]);
            debug_output.Store(988, _426[226]);
            debug_output.Store(992, _426[227]);
            debug_output.Store(996, _426[228]);
            debug_output.Store(1000, _426[229]);
            debug_output.Store(1004, _426[230]);
            debug_output.Store(1008, _426[231]);
            debug_output.Store(1012, _426[232]);
            debug_output.Store(1016, _426[233]);
            debug_output.Store(1020, _426[234]);
            debug_output.Store(1024, _426[235]);
            debug_output.Store(1028, _426[236]);
            debug_output.Store(1032, _426[237]);
            debug_output.Store(1036, _426[238]);
            debug_output.Store(1040, _426[239]);
            debug_output.Store(1044, _426[240]);
            debug_output.Store(1048, _426[241]);
            debug_output.Store(1052, _426[242]);
            debug_output.Store(1056, _426[243]);
            debug_output.Store(1060, _426[244]);
            debug_output.Store(1064, _426[245]);
            debug_output.Store(1068, _426[246]);
            debug_output.Store(1072, _426[247]);
            debug_output.Store(1076, _426[248]);
            debug_output.Store(1080, _426[249]);
            debug_output.Store(1084, _426[250]);
            debug_output.Store(1088, _426[251]);
            debug_output.Store(1092, _426[252]);
            debug_output.Store(1096, _426[253]);
            debug_output.Store(1100, _426[254]);
            debug_output.Store(1104, _426[255]);
        }
    }
}

[numthreads(256, 1, 1)]
void main(SPIRV_Cross_Input stage_input)
{
    gl_LocalInvocationIndex = stage_input.gl_LocalInvocationIndex;
    comp_main();
}
    d{      #pragma clang diagnostic ignored "-Wmissing-prototypes"
#pragma clang diagnostic ignored "-Wmissing-braces"

#include <metal_stdlib>
#include <simd/simd.h>

using namespace metal;

template<typename T, size_t Num>
struct spvUnsafeArray
{
    T elements[Num ? Num : 1];
    
    thread T& operator [] (size_t pos) thread
    {
        return elements[pos];
    }
    constexpr const thread T& operator [] (size_t pos) const thread
    {
        return elements[pos];
    }
    
    device T& operator [] (size_t pos) device
    {
        return elements[pos];
    }
    constexpr const device T& operator [] (size_t pos) const device
    {
        return elements[pos];
    }
    
    constexpr const constant T& operator [] (size_t pos) const constant
    {
        return elements[pos];
    }
    
    threadgroup T& operator [] (size_t pos) threadgroup
    {
        return elements[pos];
    }
    constexpr const threadgroup T& operator [] (size_t pos) const threadgroup
    {
        return elements[pos];
    }
};

struct HistogramData
{
    uint data[256];
};

struct AverageHistogramConfig
{
    uint pixel_count;
    float min_log_luma;
    float log_luma_range;
    float dt;
    float low_percentile;
    float high_percentile;
    float low_adjust_speed;
    float high_adjust_speed;
    uint write_debug_output;
};

struct HistogramResult
{
    float average_luminosity_interpolated;
    float average_luminosity_this_frame;
    float average_luminosity_last_frame;
    float min_luminosity_interpolated;
    float min_luminosity_this_frame;
    float min_luminosity_last_frame;
    float max_luminosity_interpolated;
    float max_luminosity_this_frame;
    float max_luminosity_last_frame;
    float low_luminosity_interpolated;
    float low_luminosity_this_frame;
    float low_luminosity_last_frame;
    float high_luminosity_interpolated;
    float high_luminosity_this_frame;
    float high_luminosity_last_frame;
    float average_bin_include_zero;
    float average_bin_non_zero;
    uint min_bin;
    uint max_bin;
    uint low_bin;
    uint high_bin;
};

struct HistogramResultBuffer
{
    HistogramResult result;
};

struct DebugOutput
{
    HistogramResult result;
    uint data[256];
};

constant uint3 gl_WorkGroupSize [[maybe_unused]] = uint3(256u, 1u, 1u);

struct spvDescriptorSetBuffer0
{
    device HistogramData* histogram_data [[id(0)]];
    constant AverageHistogramConfig* config [[id(1)]];
    device HistogramResultBuffer* histogram_result [[id(2)]];
    device DebugOutput* debug_output [[id(3)]];
};

template<typename T, uint A>
inline void spvArrayCopyFromConstantToStack1(thread T (&dst)[A], constant T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromConstantToThreadGroup1(threadgroup T (&dst)[A], constant T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromStackToStack1(thread T (&dst)[A], thread const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromStackToThreadGroup1(threadgroup T (&dst)[A], thread const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromThreadGroupToStack1(thread T (&dst)[A], threadgroup const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromThreadGroupToThreadGroup1(threadgroup T (&dst)[A], threadgroup const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromDeviceToDevice1(device T (&dst)[A], device const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromConstantToDevice1(device T (&dst)[A], constant T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromStackToDevice1(device T (&dst)[A], thread const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromThreadGroupToDevice1(device T (&dst)[A], threadgroup const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromDeviceToStack1(thread T (&dst)[A], device const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

template<typename T, uint A>
inline void spvArrayCopyFromDeviceToThreadGroup1(threadgroup T (&dst)[A], device const T (&src)[A])
{
    for (uint i = 0; i < A; i++)
    {
        dst[i] = src[i];
    }
}

static inline __attribute__((always_inline))
float bin_to_luminosity(thread const float& bin, thread const float& min_log_luma, thread const float& log_luma_range)
{
    return exp2(((bin / 255.0) * log_luma_range) + min_log_luma);
}

kernel void main0(constant spvDescriptorSetBuffer0& spvDescriptorSet0 [[buffer(0)]], uint gl_LocalInvocationIndex [[thread_index_in_threadgroup]])
{
    threadgroup float HistogramShared[256];
    float count_for_this_bin = float((*spvDescriptorSet0.histogram_data).data[gl_LocalInvocationIndex]);
    HistogramShared[gl_LocalInvocationIndex] = count_for_this_bin * float(gl_LocalInvocationIndex + 1u);
    threadgroup_barrier(mem_flags::mem_threadgroup);
    uint histogram_sample_index = 128u;
    for (;;)
    {
        if (histogram_sample_index > 0u)
        {
            if (gl_LocalInvocationIndex < histogram_sample_index)
            {
                HistogramShared[gl_LocalInvocationIndex] += HistogramShared[gl_LocalInvocationIndex + histogram_sample_index];
            }
            threadgroup_barrier(mem_flags::mem_threadgroup);
            histogram_sample_index = histogram_sample_index >> uint(1);
            continue;
        }
        else
        {
            break;
        }
    }
    if (gl_LocalInvocationIndex == 0u)
    {
        uint zero_pixel_count = uint(count_for_this_bin);
        uint max_bin = 0u;
        uint min_bin = 255u;
        uint pixels_seen = 0u;
        uint high_bin = 0u;
        uint high_pixel_thresh = uint((1.0 - (*spvDescriptorSet0.config).high_percentile) * float((*spvDescriptorSet0.config).pixel_count - zero_pixel_count));
        uint low_bin = 0u;
        uint low_pixel_thresh = uint((1.0 - (*spvDescriptorSet0.config).low_percentile) * float((*spvDescriptorSet0.config).pixel_count - zero_pixel_count));
        int i = 255;
        for (;;)
        {
            if (i >= 0)
            {
                uint data = (*spvDescriptorSet0.histogram_data).data[i];
                if (data > 0u)
                {
                    max_bin = max(max_bin, uint(i));
                    min_bin = uint(i);
                }
                pixels_seen += data;
                if (pixels_seen > high_pixel_thresh)
                {
                    high_bin = max(high_bin, uint(i));
                }
                if (pixels_seen > low_pixel_thresh)
                {
                    low_bin = max(low_bin, uint(i));
                }
                i--;
                continue;
            }
            else
            {
                break;
            }
        }
        float average_bin_include_zero = (HistogramShared[0] / float((*spvDescriptorSet0.config).pixel_count)) - 1.0;
        float non_zero_pixel_count = float((*spvDescriptorSet0.config).pixel_count) - float(zero_pixel_count);
        float average_bin_non_zero = (HistogramShared[0] - non_zero_pixel_count) / fast::max(non_zero_pixel_count, 1.0);
        float average_bin = average_bin_include_zero;
        (*spvDescriptorSet0.histogram_result).result.average_luminosity_last_frame = (*spvDescriptorSet0.histogram_result).result.average_luminosity_interpolated;
        (*spvDescriptorSet0.histogram_result).result.min_luminosity_last_frame = (*spvDescriptorSet0.histogram_result).result.min_luminosity_interpolated;
        (*spvDescriptorSet0.histogram_result).result.max_luminosity_last_frame = (*spvDescriptorSet0.histogram_result).result.max_luminosity_interpolated;
        (*spvDescriptorSet0.histogram_result).result.low_luminosity_last_frame = (*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated;
        (*spvDescriptorSet0.histogram_result).result.high_luminosity_last_frame = (*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated;
        float param = average_bin;
        float param_1 = (*spvDescriptorSet0.config).min_log_luma;
        float param_2 = (*spvDescriptorSet0.config).log_luma_range;
        (*spvDescriptorSet0.histogram_result).result.average_luminosity_this_frame = bin_to_luminosity(param, param_1, param_2);
        float param_3 = float(min_bin);
        float param_4 = (*spvDescriptorSet0.config).min_log_luma;
        float param_5 = (*spvDescriptorSet0.config).log_luma_range;
        (*spvDescriptorSet0.histogram_result).result.min_luminosity_this_frame = bin_to_luminosity(param_3, param_4, param_5);
        float param_6 = float(max_bin);
        float param_7 = (*spvDescriptorSet0.config).min_log_luma;
        float param_8 = (*spvDescriptorSet0.config).log_luma_range;
        (*spvDescriptorSet0.histogram_result).result.max_luminosity_this_frame = bin_to_luminosity(param_6, param_7, param_8);
        float param_9 = float(low_bin);
        float param_10 = (*spvDescriptorSet0.config).min_log_luma;
        float param_11 = (*spvDescriptorSet0.config).log_luma_range;
        (*spvDescriptorSet0.histogram_result).result.low_luminosity_this_frame = bin_to_luminosity(param_9, param_10, param_11);
        float param_12 = float(high_bin);
        float param_13 = (*spvDescriptorSet0.config).min_log_luma;
        float param_14 = (*spvDescriptorSet0.config).log_luma_range;
        (*spvDescriptorSet0.histogram_result).result.high_luminosity_this_frame = bin_to_luminosity(param_12, param_13, param_14);
        float interp_high = fast::clamp(1.0 - exp((-(*spvDescriptorSet0.config).high_adjust_speed) * (*spvDescriptorSet0.config).dt), 0.0, 1.0);
        float interp_low = fast::clamp(1.0 - exp((-(*spvDescriptorSet0.config).low_adjust_speed) * (*spvDescriptorSet0.config).dt), 0.0, 1.0);
        (*spvDescriptorSet0.histogram_result).result.average_luminosity_interpolated = mix((*spvDescriptorSet0.histogram_result).result.average_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.average_luminosity_this_frame, ((*spvDescriptorSet0.histogram_result).result.average_luminosity_interpolated < (*spvDescriptorSet0.histogram_result).result.average_luminosity_this_frame) ? interp_high : interp_low);
        (*spvDescriptorSet0.histogram_result).result.min_luminosity_interpolated = mix((*spvDescriptorSet0.histogram_result).result.min_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.min_luminosity_this_frame, ((*spvDescriptorSet0.histogram_result).result.min_luminosity_interpolated < (*spvDescriptorSet0.histogram_result).result.min_luminosity_this_frame) ? interp_high : interp_low);
        (*spvDescriptorSet0.histogram_result).result.max_luminosity_interpolated = mix((*spvDescriptorSet0.histogram_result).result.max_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.max_luminosity_this_frame, ((*spvDescriptorSet0.histogram_result).result.max_luminosity_interpolated < (*spvDescriptorSet0.histogram_result).result.max_luminosity_this_frame) ? interp_high : interp_low);
        (*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated = mix((*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.low_luminosity_this_frame, ((*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated < (*spvDescriptorSet0.histogram_result).result.low_luminosity_this_frame) ? interp_high : interp_low);
        (*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated = mix((*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.high_luminosity_this_frame, ((*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated < (*spvDescriptorSet0.histogram_result).result.high_luminosity_this_frame) ? interp_high : interp_low);
        (*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated = fast::max((*spvDescriptorSet0.histogram_result).result.min_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated);
        (*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated = fast::max((*spvDescriptorSet0.histogram_result).result.low_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated);
        (*spvDescriptorSet0.histogram_result).result.max_luminosity_interpolated = fast::max((*spvDescriptorSet0.histogram_result).result.high_luminosity_interpolated, (*spvDescriptorSet0.histogram_result).result.max_luminosity_interpolated);
        (*spvDescriptorSet0.histogram_result).result.average_bin_include_zero = average_bin_include_zero;
        (*spvDescriptorSet0.histogram_result).result.average_bin_non_zero = average_bin_non_zero;
        (*spvDescriptorSet0.histogram_result).result.min_bin = min_bin;
        (*spvDescriptorSet0.histogram_result).result.max_bin = max_bin;
        (*spvDescriptorSet0.histogram_result).result.low_bin = low_bin;
        (*spvDescriptorSet0.histogram_result).result.high_bin = high_bin;
        if ((*spvDescriptorSet0.config).write_debug_output != 0u)
        {
            (*spvDescriptorSet0.debug_output).result = (*spvDescriptorSet0.histogram_result).result;
            spvUnsafeArray<uint, 256> _426;
            spvArrayCopyFromDeviceToStack1(_426.elements, (*spvDescriptorSet0.histogram_data).data);
            (*spvDescriptorSet0.debug_output).data[0] = _426[0];
            (*spvDescriptorSet0.debug_output).data[1] = _426[1];
            (*spvDescriptorSet0.debug_output).data[2] = _426[2];
            (*spvDescriptorSet0.debug_output).data[3] = _426[3];
            (*spvDescriptorSet0.debug_output).data[4] = _426[4];
            (*spvDescriptorSet0.debug_output).data[5] = _426[5];
            (*spvDescriptorSet0.debug_output).data[6] = _426[6];
            (*spvDescriptorSet0.debug_output).data[7] = _426[7];
            (*spvDescriptorSet0.debug_output).data[8] = _426[8];
            (*spvDescriptorSet0.debug_output).data[9] = _426[9];
            (*spvDescriptorSet0.debug_output).data[10] = _426[10];
            (*spvDescriptorSet0.debug_output).data[11] = _426[11];
            (*spvDescriptorSet0.debug_output).data[12] = _426[12];
            (*spvDescriptorSet0.debug_output).data[13] = _426[13];
            (*spvDescriptorSet0.debug_output).data[14] = _426[14];
            (*spvDescriptorSet0.debug_output).data[15] = _426[15];
            (*spvDescriptorSet0.debug_output).data[16] = _426[16];
            (*spvDescriptorSet0.debug_output).data[17] = _426[17];
            (*spvDescriptorSet0.debug_output).data[18] = _426[18];
            (*spvDescriptorSet0.debug_output).data[19] = _426[19];
            (*spvDescriptorSet0.debug_output).data[20] = _426[20];
            (*spvDescriptorSet0.debug_output).data[21] = _426[21];
            (*spvDescriptorSet0.debug_output).data[22] = _426[22];
            (*spvDescriptorSet0.debug_output).data[23] = _426[23];
            (*spvDescriptorSet0.debug_output).data[24] = _426[24];
            (*spvDescriptorSet0.debug_output).data[25] = _426[25];
            (*spvDescriptorSet0.debug_output).data[26] = _426[26];
            (*spvDescriptorSet0.debug_output).data[27] = _426[27];
            (*spvDescriptorSet0.debug_output).data[28] = _426[28];
            (*spvDescriptorSet0.debug_output).data[29] = _426[29];
            (*spvDescriptorSet0.debug_output).data[30] = _426[30];
            (*spvDescriptorSet0.debug_output).data[31] = _426[31];
            (*spvDescriptorSet0.debug_output).data[32] = _426[32];
            (*spvDescriptorSet0.debug_output).data[33] = _426[33];
            (*spvDescriptorSet0.debug_output).data[34] = _426[34];
            (*spvDescriptorSet0.debug_output).data[35] = _426[35];
            (*spvDescriptorSet0.debug_output).data[36] = _426[36];
            (*spvDescriptorSet0.debug_output).data[37] = _426[37];
            (*spvDescriptorSet0.debug_output).data[38] = _426[38];
            (*spvDescriptorSet0.debug_output).data[39] = _426[39];
            (*spvDescriptorSet0.debug_output).data[40] = _426[40];
            (*spvDescriptorSet0.debug_output).data[41] = _426[41];
            (*spvDescriptorSet0.debug_output).data[42] = _426[42];
            (*spvDescriptorSet0.debug_output).data[43] = _426[43];
            (*spvDescriptorSet0.debug_output).data[44] = _426[44];
            (*spvDescriptorSet0.debug_output).data[45] = _426[45];
            (*spvDescriptorSet0.debug_output).data[46] = _426[46];
            (*spvDescriptorSet0.debug_output).data[47] = _426[47];
            (*spvDescriptorSet0.debug_output).data[48] = _426[48];
            (*spvDescriptorSet0.debug_output).data[49] = _426[49];
            (*spvDescriptorSet0.debug_output).data[50] = _426[50];
            (*spvDescriptorSet0.debug_output).data[51] = _426[51];
            (*spvDescriptorSet0.debug_output).data[52] = _426[52];
            (*spvDescriptorSet0.debug_output).data[53] = _426[53];
            (*spvDescriptorSet0.debug_output).data[54] = _426[54];
            (*spvDescriptorSet0.debug_output).data[55] = _426[55];
            (*spvDescriptorSet0.debug_output).data[56] = _426[56];
            (*spvDescriptorSet0.debug_output).data[57] = _426[57];
            (*spvDescriptorSet0.debug_output).data[58] = _426[58];
            (*spvDescriptorSet0.debug_output).data[59] = _426[59];
            (*spvDescriptorSet0.debug_output).data[60] = _426[60];
            (*spvDescriptorSet0.debug_output).data[61] = _426[61];
            (*spvDescriptorSet0.debug_output).data[62] = _426[62];
            (*spvDescriptorSet0.debug_output).data[63] = _426[63];
            (*spvDescriptorSet0.debug_output).data[64] = _426[64];
            (*spvDescriptorSet0.debug_output).data[65] = _426[65];
            (*spvDescriptorSet0.debug_output).data[66] = _426[66];
            (*spvDescriptorSet0.debug_output).data[67] = _426[67];
            (*spvDescriptorSet0.debug_output).data[68] = _426[68];
            (*spvDescriptorSet0.debug_output).data[69] = _426[69];
            (*spvDescriptorSet0.debug_output).data[70] = _426[70];
            (*spvDescriptorSet0.debug_output).data[71] = _426[71];
            (*spvDescriptorSet0.debug_output).data[72] = _426[72];
            (*spvDescriptorSet0.debug_output).data[73] = _426[73];
            (*spvDescriptorSet0.debug_output).data[74] = _426[74];
            (*spvDescriptorSet0.debug_output).data[75] = _426[75];
            (*spvDescriptorSet0.debug_output).data[76] = _426[76];
            (*spvDescriptorSet0.debug_output).data[77] = _426[77];
            (*spvDescriptorSet0.debug_output).data[78] = _426[78];
            (*spvDescriptorSet0.debug_output).data[79] = _426[79];
            (*spvDescriptorSet0.debug_output).data[80] = _426[80];
            (*spvDescriptorSet0.debug_output).data[81] = _426[81];
            (*spvDescriptorSet0.debug_output).data[82] = _426[82];
            (*spvDescriptorSet0.debug_output).data[83] = _426[83];
            (*spvDescriptorSet0.debug_output).data[84] = _426[84];
            (*spvDescriptorSet0.debug_output).data[85] = _426[85];
            (*spvDescriptorSet0.debug_output).data[86] = _426[86];
            (*spvDescriptorSet0.debug_output).data[87] = _426[87];
            (*spvDescriptorSet0.debug_output).data[88] = _426[88];
            (*spvDescriptorSet0.debug_output).data[89] = _426[89];
            (*spvDescriptorSet0.debug_output).data[90] = _426[90];
            (*spvDescriptorSet0.debug_output).data[91] = _426[91];
            (*spvDescriptorSet0.debug_output).data[92] = _426[92];
            (*spvDescriptorSet0.debug_output).data[93] = _426[93];
            (*spvDescriptorSet0.debug_output).data[94] = _426[94];
            (*spvDescriptorSet0.debug_output).data[95] = _426[95];
            (*spvDescriptorSet0.debug_output).data[96] = _426[96];
            (*spvDescriptorSet0.debug_output).data[97] = _426[97];
            (*spvDescriptorSet0.debug_output).data[98] = _426[98];
            (*spvDescriptorSet0.debug_output).data[99] = _426[99];
            (*spvDescriptorSet0.debug_output).data[100] = _426[100];
            (*spvDescriptorSet0.debug_output).data[101] = _426[101];
            (*spvDescriptorSet0.debug_output).data[102] = _426[102];
            (*spvDescriptorSet0.debug_output).data[103] = _426[103];
            (*spvDescriptorSet0.debug_output).data[104] = _426[104];
            (*spvDescriptorSet0.debug_output).data[105] = _426[105];
            (*spvDescriptorSet0.debug_output).data[106] = _426[106];
            (*spvDescriptorSet0.debug_output).data[107] = _426[107];
            (*spvDescriptorSet0.debug_output).data[108] = _426[108];
            (*spvDescriptorSet0.debug_output).data[109] = _426[109];
            (*spvDescriptorSet0.debug_output).data[110] = _426[110];
            (*spvDescriptorSet0.debug_output).data[111] = _426[111];
            (*spvDescriptorSet0.debug_output).data[112] = _426[112];
            (*spvDescriptorSet0.debug_output).data[113] = _426[113];
            (*spvDescriptorSet0.debug_output).data[114] = _426[114];
            (*spvDescriptorSet0.debug_output).data[115] = _426[115];
            (*spvDescriptorSet0.debug_output).data[116] = _426[116];
            (*spvDescriptorSet0.debug_output).data[117] = _426[117];
            (*spvDescriptorSet0.debug_output).data[118] = _426[118];
            (*spvDescriptorSet0.debug_output).data[119] = _426[119];
            (*spvDescriptorSet0.debug_output).data[120] = _426[120];
            (*spvDescriptorSet0.debug_output).data[121] = _426[121];
            (*spvDescriptorSet0.debug_output).data[122] = _426[122];
            (*spvDescriptorSet0.debug_output).data[123] = _426[123];
            (*spvDescriptorSet0.debug_output).data[124] = _426[124];
            (*spvDescriptorSet0.debug_output).data[125] = _426[125];
            (*spvDescriptorSet0.debug_output).data[126] = _426[126];
            (*spvDescriptorSet0.debug_output).data[127] = _426[127];
            (*spvDescriptorSet0.debug_output).data[128] = _426[128];
            (*spvDescriptorSet0.debug_output).data[129] = _426[129];
            (*spvDescriptorSet0.debug_output).data[130] = _426[130];
            (*spvDescriptorSet0.debug_output).data[131] = _426[131];
            (*spvDescriptorSet0.debug_output).data[132] = _426[132];
            (*spvDescriptorSet0.debug_output).data[133] = _426[133];
            (*spvDescriptorSet0.debug_output).data[134] = _426[134];
            (*spvDescriptorSet0.debug_output).data[135] = _426[135];
            (*spvDescriptorSet0.debug_output).data[136] = _426[136];
            (*spvDescriptorSet0.debug_output).data[137] = _426[137];
            (*spvDescriptorSet0.debug_output).data[138] = _426[138];
            (*spvDescriptorSet0.debug_output).data[139] = _426[139];
            (*spvDescriptorSet0.debug_output).data[140] = _426[140];
            (*spvDescriptorSet0.debug_output).data[141] = _426[141];
            (*spvDescriptorSet0.debug_output).data[142] = _426[142];
            (*spvDescriptorSet0.debug_output).data[143] = _426[143];
            (*spvDescriptorSet0.debug_output).data[144] = _426[144];
            (*spvDescriptorSet0.debug_output).data[145] = _426[145];
            (*spvDescriptorSet0.debug_output).data[146] = _426[146];
            (*spvDescriptorSet0.debug_output).data[147] = _426[147];
            (*spvDescriptorSet0.debug_output).data[148] = _426[148];
            (*spvDescriptorSet0.debug_output).data[149] = _426[149];
            (*spvDescriptorSet0.debug_output).data[150] = _426[150];
            (*spvDescriptorSet0.debug_output).data[151] = _426[151];
            (*spvDescriptorSet0.debug_output).data[152] = _426[152];
            (*spvDescriptorSet0.debug_output).data[153] = _426[153];
            (*spvDescriptorSet0.debug_output).data[154] = _426[154];
            (*spvDescriptorSet0.debug_output).data[155] = _426[155];
            (*spvDescriptorSet0.debug_output).data[156] = _426[156];
            (*spvDescriptorSet0.debug_output).data[157] = _426[157];
            (*spvDescriptorSet0.debug_output).data[158] = _426[158];
            (*spvDescriptorSet0.debug_output).data[159] = _426[159];
            (*spvDescriptorSet0.debug_output).data[160] = _426[160];
            (*spvDescriptorSet0.debug_output).data[161] = _426[161];
            (*spvDescriptorSet0.debug_output).data[162] = _426[162];
            (*spvDescriptorSet0.debug_output).data[163] = _426[163];
            (*spvDescriptorSet0.debug_output).data[164] = _426[164];
            (*spvDescriptorSet0.debug_output).data[165] = _426[165];
            (*spvDescriptorSet0.debug_output).data[166] = _426[166];
            (*spvDescriptorSet0.debug_output).data[167] = _426[167];
            (*spvDescriptorSet0.debug_output).data[168] = _426[168];
            (*spvDescriptorSet0.debug_output).data[169] = _426[169];
            (*spvDescriptorSet0.debug_output).data[170] = _426[170];
            (*spvDescriptorSet0.debug_output).data[171] = _426[171];
            (*spvDescriptorSet0.debug_output).data[172] = _426[172];
            (*spvDescriptorSet0.debug_output).data[173] = _426[173];
            (*spvDescriptorSet0.debug_output).data[174] = _426[174];
            (*spvDescriptorSet0.debug_output).data[175] = _426[175];
            (*spvDescriptorSet0.debug_output).data[176] = _426[176];
            (*spvDescriptorSet0.debug_output).data[177] = _426[177];
            (*spvDescriptorSet0.debug_output).data[178] = _426[178];
            (*spvDescriptorSet0.debug_output).data[179] = _426[179];
            (*spvDescriptorSet0.debug_output).data[180] = _426[180];
            (*spvDescriptorSet0.debug_output).data[181] = _426[181];
            (*spvDescriptorSet0.debug_output).data[182] = _426[182];
            (*spvDescriptorSet0.debug_output).data[183] = _426[183];
            (*spvDescriptorSet0.debug_output).data[184] = _426[184];
            (*spvDescriptorSet0.debug_output).data[185] = _426[185];
            (*spvDescriptorSet0.debug_output).data[186] = _426[186];
            (*spvDescriptorSet0.debug_output).data[187] = _426[187];
            (*spvDescriptorSet0.debug_output).data[188] = _426[188];
            (*spvDescriptorSet0.debug_output).data[189] = _426[189];
            (*spvDescriptorSet0.debug_output).data[190] = _426[190];
            (*spvDescriptorSet0.debug_output).data[191] = _426[191];
            (*spvDescriptorSet0.debug_output).data[192] = _426[192];
            (*spvDescriptorSet0.debug_output).data[193] = _426[193];
            (*spvDescriptorSet0.debug_output).data[194] = _426[194];
            (*spvDescriptorSet0.debug_output).data[195] = _426[195];
            (*spvDescriptorSet0.debug_output).data[196] = _426[196];
            (*spvDescriptorSet0.debug_output).data[197] = _426[197];
            (*spvDescriptorSet0.debug_output).data[198] = _426[198];
            (*spvDescriptorSet0.debug_output).data[199] = _426[199];
            (*spvDescriptorSet0.debug_output).data[200] = _426[200];
            (*spvDescriptorSet0.debug_output).data[201] = _426[201];
            (*spvDescriptorSet0.debug_output).data[202] = _426[202];
            (*spvDescriptorSet0.debug_output).data[203] = _426[203];
            (*spvDescriptorSet0.debug_output).data[204] = _426[204];
            (*spvDescriptorSet0.debug_output).data[205] = _426[205];
            (*spvDescriptorSet0.debug_output).data[206] = _426[206];
            (*spvDescriptorSet0.debug_output).data[207] = _426[207];
            (*spvDescriptorSet0.debug_output).data[208] = _426[208];
            (*spvDescriptorSet0.debug_output).data[209] = _426[209];
            (*spvDescriptorSet0.debug_output).data[210] = _426[210];
            (*spvDescriptorSet0.debug_output).data[211] = _426[211];
            (*spvDescriptorSet0.debug_output).data[212] = _426[212];
            (*spvDescriptorSet0.debug_output).data[213] = _426[213];
            (*spvDescriptorSet0.debug_output).data[214] = _426[214];
            (*spvDescriptorSet0.debug_output).data[215] = _426[215];
            (*spvDescriptorSet0.debug_output).data[216] = _426[216];
            (*spvDescriptorSet0.debug_output).data[217] = _426[217];
            (*spvDescriptorSet0.debug_output).data[218] = _426[218];
            (*spvDescriptorSet0.debug_output).data[219] = _426[219];
            (*spvDescriptorSet0.debug_output).data[220] = _426[220];
            (*spvDescriptorSet0.debug_output).data[221] = _426[221];
            (*spvDescriptorSet0.debug_output).data[222] = _426[222];
            (*spvDescriptorSet0.debug_output).data[223] = _426[223];
            (*spvDescriptorSet0.debug_output).data[224] = _426[224];
            (*spvDescriptorSet0.debug_output).data[225] = _426[225];
            (*spvDescriptorSet0.debug_output).data[226] = _426[226];
            (*spvDescriptorSet0.debug_output).data[227] = _426[227];
            (*spvDescriptorSet0.debug_output).data[228] = _426[228];
            (*spvDescriptorSet0.debug_output).data[229] = _426[229];
            (*spvDescriptorSet0.debug_output).data[230] = _426[230];
            (*spvDescriptorSet0.debug_output).data[231] = _426[231];
            (*spvDescriptorSet0.debug_output).data[232] = _426[232];
            (*spvDescriptorSet0.debug_output).data[233] = _426[233];
            (*spvDescriptorSet0.debug_output).data[234] = _426[234];
            (*spvDescriptorSet0.debug_output).data[235] = _426[235];
            (*spvDescriptorSet0.debug_output).data[236] = _426[236];
            (*spvDescriptorSet0.debug_output).data[237] = _426[237];
            (*spvDescriptorSet0.debug_output).data[238] = _426[238];
            (*spvDescriptorSet0.debug_output).data[239] = _426[239];
            (*spvDescriptorSet0.debug_output).data[240] = _426[240];
            (*spvDescriptorSet0.debug_output).data[241] = _426[241];
            (*spvDescriptorSet0.debug_output).data[242] = _426[242];
            (*spvDescriptorSet0.debug_output).data[243] = _426[243];
            (*spvDescriptorSet0.debug_output).data[244] = _426[244];
            (*spvDescriptorSet0.debug_output).data[245] = _426[245];
            (*spvDescriptorSet0.debug_output).data[246] = _426[246];
            (*spvDescriptorSet0.debug_output).data[247] = _426[247];
            (*spvDescriptorSet0.debug_output).data[248] = _426[248];
            (*spvDescriptorSet0.debug_output).data[249] = _426[249];
            (*spvDescriptorSet0.debug_output).data[250] = _426[250];
            (*spvDescriptorSet0.debug_output).data[251] = _426[251];
            (*spvDescriptorSet0.debug_output).data[252] = _426[252];
            (*spvDescriptorSet0.debug_output).data[253] = _426[253];
            (*spvDescriptorSet0.debug_output).data[254] = _426[254];
            (*spvDescriptorSet0.debug_output).data[255] = _426[255];
        }
    }
}

    �a      #     �                GLSL.std.450                     main    "                    G           H         #       G        G     "       G     !       G  "         H  b       #       H  b      #      H  b      #      H  b      #      H  b      #      H  b      #      H  b      #      H  b      #      H  b      #       G  b      G  d   "       G  d   !      H  �       #       H  �      #      H  �      #      H  �      #      H  �      #      H  �      #      H  �      #      H  �      #      H  �      #       H  �   	   #   $   H  �   
   #   (   H  �      #   ,   H  �      #   0   H  �      #   4   H  �      #   8   H  �      #   <   H  �      #   @   H  �      #   D   H  �      #   H   H  �      #   L   H  �      #   P   H  �       #       G  �      G  �   "       G  �   !      G  �        H  �      #       H  �     #   T   G  �     G  �  "       G  �  !      G  �             !                              +                                          ;                       +                !         ;  !   "         $           (            )      (   ;  )   *      +     .         2         +     4      +     5     +     8   �   +     ?         @   +     Q      +     ]   �   +     a     �?  b                                 c      b   ;  c   d      +     e         f         +     s      +     �   �     �                                                                    �   �      �      �   ;  �   �      +     �      +     �      +     �      +     �      +     �      +     �   	   +     �      +     �      +     �      +       
   +          +     #      +     �     +     �     +     �     +     �     +     �     +     �       �          �  �   �     �     �  ;  �  �        �     �      �        +     �     +     �     +     �     +     �     +     �     +     �     +     �     +     �     +     �     +     �     +     �     +     �      +     �  !   +     �  "   +        #   +       $   +       %   +     	  &   +       '   +       (   +       )   +       *   +       +   +       ,   +       -   +     !  .   +     $  /   +     '  0   +     *  1   +     -  2   +     0  3   +     3  4   +     6  5   +     9  6   +     <  7   +     ?  8   +     B  9   +     E  :   +     H  ;   +     K  <   +     N  =   +     Q  >   +     T  ?   +     W  @   +     Z  A   +     ]  B   +     `  C   +     c  D   +     f  E   +     i  F   +     l  G   +     o  H   +     r  I   +     u  J   +     x  K   +     {  L   +     ~  M   +     �  N   +     �  O   +     �  P   +     �  Q   +     �  R   +     �  S   +     �  T   +     �  U   +     �  V   +     �  W   +     �  X   +     �  Y   +     �  Z   +     �  [   +     �  \   +     �  ]   +     �  ^   +     �  _   +     �  `   +     �  a   +     �  b   +     �  c   +     �  d   +     �  e   +     �  f   +     �  g   +     �  h   +     �  i   +     �  j   +     �  k   +     �  l   +     �  m   +     �  n   +     �  o   +     �  p   +     �  q   +     �  r   +     �  s   +     �  t   +     �  u   +     �  v   +     �  w   +     �  x   +       y   +       z   +       {   +       |   +       }   +       ~   +          +       �   +       �   +       �   +        �   +     #  �   +     &  �   +     )  �   +     ,  �   +     /  �   +     2  �   +     5  �   +     8  �   +     ;  �   +     >  �   +     A  �   +     D  �   +     G  �   +     J  �   +     M  �   +     P  �   +     S  �   +     V  �   +     Y  �   +     \  �   +     _  �   +     b  �   +     e  �   +     h  �   +     k  �   +     n  �   +     q  �   +     t  �   +     w  �   +     z  �   +     }  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +       �   +       �   +       �   +     
  �   +       �   +       �   +       �   +       �   +       �   +       �   +       �   +     "  �   +     %  �   +     (  �   +     +  �   +     .  �   +     1  �   +     4  �   +     7  �   +     :  �   +     =  �   +     @  �   +     C  �   +     F  �   +     I  �   +     L  �   +     O  �   +     R  �   +     U  �   +     X  �   +     [  �   +     ^  �   +     a  �   +     d  �   +     g  �   +     j  �   +     m  �   +     p  �   +     s  �   +     v  �   +     y  �   +     |  �   +       �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �   +     �  �     �        ,  �  �     .   .   +     �  ���;6               �     =     #   "   A  $   %          #   =     &   %   p     '   &   �     /   #   .   p     0   /   �     1   '   0   A  2   3   *   #   >  3   1   �  4   4   5   �  9   �  9   �     �  8      S   <   �  @   A   �  ?   �  ;   <      �  A   :   ;   �  :   �  @   D   #   �  �  F       �  D   E   F   �  E   �     J   #   �  A  2   K   *   J   =     L   K   =     N   3   �     O   N   L   >  3   O   �  F   �  F   �  4   4   5   �  <   �  <   �     S   �  Q   �  9   �  ;   �  @   U   #   ?   �  W       �  U   V   W   �  V   m     Z   '   A  f   g   d   e   =     h   g   �     i   a   h   A  $   j   d       =     k   j   �     m   k   Z   p     n   m   �     o   i   n   m     p   o   A  f   t   d   s   =     u   t   �     v   a   u   �     |   v   n   m     }   |   �  �   �  �   �     �  ?   V   �   �   �     �  ?   V   �  �   �     �  ?   V   �  �   �     �  ?   V   �  �   �     �  ]   V   �  �   �     �  �   V   �   �   �  @   �   �      �  �   �       �  �   �   �   �  �   A  $   �          �  =     �   �   �  @   �   �   ?   �  �       �  �   �   �   �  �   |     �   �       �      )   �  �   �  �   �  �   �     �  �  �   �   �   �     �  �  �   �   �   �     �   �  �   �  @   �   �   p   �  �       �  �   �   �   �  �   |     �   �       �      )   �  �   �  �   �  �   �     �  �  �   �   �   �  @   �   �   }   �  �       �  �   �   �   �  �   |     �   �       �      )   �  �   �  �   �  �   �     �  �  �   �   �   �  �   �  �   �     �   �  Q   �  �   �  �   A  2   �   *       =     �   �   p     �   k   �     �   �   �   �     �   �   a   p     �   Z   �     �   �   �   =     �   �   �     �   �   �        �      (   �   a   �     �   �   �   A  f   �   �           =     �   �   A  f   �   �       �   >  �   �   A  f   �   �       �   =     �   �   A  f   �   �       e   >  �   �   A  f   �   �       �   =     �   �   A  f   �   �       �   >  �   �   A  f   �   �       �   =     �   �   A  f   �   �       �   >  �   �   A  f   �   �       �   =     �   �   A  f   �   �       �   >  �   �   A  f   �   d   Q   =     �   �   A  f   �   d   �   =     �   �   �     �  �   �       �     2   �  �   �        �        �  A  f   �   �       Q   >  �   �  p     �   �  �     �  �   �       �     2   �  �   �        �        �  A  f   �   �       s   >  �   �  p     �   �  �     �  �   �       �     2   �  �   �        �        �  A  f     �       �   >    �  p       �  �     �    �       �     2   �  �   �        �        �  A  f     �         >    �  p       �  �     �    �       �     2   �  �   �        �        �  A  f     �         >    �  A  f     d   �   =                  A  f     d   �   =         �                 !           �     "  a   !       $     +   "  #  a   A  f   &  d   �   =     '  &       (  '  �     +  (         ,        +  �     -  a   ,       .     +   -  #  a   =     0  �   =     2  �   =     4  �   =     6  �   �  @   7  4  6  �     :  7  $  .       ;     .   0  2  :  >  �   ;  =     >  �   =     @  �   =     B  �   =     D  �   �  @   E  B  D  �     H  E  $  .       I     .   >  @  H  >  �   I  =     L  �   =     N    =     P  �   =     R    �  @   S  P  R  �     V  S  $  .       W     .   L  N  V  >  �   W  =     Z  �   =     \    =     ^  �   =     `    �  @   a  ^  `  �     d  a  $  .       e     .   Z  \  d  >  �   e  =     h  �   =     j    =     l  �   =     n    �  @   o  l  n  �     r  o  $  .       s     .   h  j  r  >  �   s  =     v  �   =     x  �        y     (   v  x  >  �   y  =     |  �   =     ~  �             (   |  ~  >  �     =     �  �   =     �  �        �     (   �  �  >  �   �  A  f   �  �       �  >  �  �   A  f   �  �       �  >  �  �   A  $   �  �       �  >  �  �  A  $   �  �       �  >  �  �  A  $   �  �       �  >  �  �  A  $   �  �       �  >  �  �  A  $   �  d   �   =     �  �  �  @   �  �  ?   �  �      �  �  �  �  �  �  A  �  �  �       =  �   �  �  A  �  �  �      >  �  �  A  �  �         =     �  �  Q     �  �      A  $   �  �  Q       >  �  �  Q     �  �     A  $   �  �  Q   Q   >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �     A  $   �  �  Q   s   >  �  �  Q     �  �     A  $   �  �  Q   e   >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �  	   A  $   �  �  Q   �   >  �  �  Q     �  �  
   A  $   �  �  Q     >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �     A  $   �  �  Q     >  �  �  Q     �  �     A  $   �  �  Q   �   >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �     A  $   �  �  Q   �  >  �  �  Q     �  �      A  $   �  �  Q   �  >  �  �  Q     �  �  !   A  $   �  �  Q   �  >  �  �  Q     �  �  "   A  $   �  �  Q   �  >  �  �  Q     �  �  #   A  $     �  Q      >    �  Q       �  $   A  $     �  Q     >      Q       �  %   A  $     �  Q     >      Q       �  &   A  $   
  �  Q   	  >  
    Q       �  '   A  $     �  Q     >      Q       �  (   A  $     �  Q     >      Q       �  )   A  $     �  Q     >      Q       �  *   A  $     �  Q     >      Q       �  +   A  $     �  Q     >      Q       �  ,   A  $     �  Q     >      Q       �  -   A  $     �  Q     >      Q        �  .   A  $   "  �  Q   !  >  "     Q     #  �  /   A  $   %  �  Q   $  >  %  #  Q     &  �  0   A  $   (  �  Q   '  >  (  &  Q     )  �  1   A  $   +  �  Q   *  >  +  )  Q     ,  �  2   A  $   .  �  Q   -  >  .  ,  Q     /  �  3   A  $   1  �  Q   0  >  1  /  Q     2  �  4   A  $   4  �  Q   3  >  4  2  Q     5  �  5   A  $   7  �  Q   6  >  7  5  Q     8  �  6   A  $   :  �  Q   9  >  :  8  Q     ;  �  7   A  $   =  �  Q   <  >  =  ;  Q     >  �  8   A  $   @  �  Q   ?  >  @  >  Q     A  �  9   A  $   C  �  Q   B  >  C  A  Q     D  �  :   A  $   F  �  Q   E  >  F  D  Q     G  �  ;   A  $   I  �  Q   H  >  I  G  Q     J  �  <   A  $   L  �  Q   K  >  L  J  Q     M  �  =   A  $   O  �  Q   N  >  O  M  Q     P  �  >   A  $   R  �  Q   Q  >  R  P  Q     S  �  ?   A  $   U  �  Q   T  >  U  S  Q     V  �  @   A  $   X  �  Q   W  >  X  V  Q     Y  �  A   A  $   [  �  Q   Z  >  [  Y  Q     \  �  B   A  $   ^  �  Q   ]  >  ^  \  Q     _  �  C   A  $   a  �  Q   `  >  a  _  Q     b  �  D   A  $   d  �  Q   c  >  d  b  Q     e  �  E   A  $   g  �  Q   f  >  g  e  Q     h  �  F   A  $   j  �  Q   i  >  j  h  Q     k  �  G   A  $   m  �  Q   l  >  m  k  Q     n  �  H   A  $   p  �  Q   o  >  p  n  Q     q  �  I   A  $   s  �  Q   r  >  s  q  Q     t  �  J   A  $   v  �  Q   u  >  v  t  Q     w  �  K   A  $   y  �  Q   x  >  y  w  Q     z  �  L   A  $   |  �  Q   {  >  |  z  Q     }  �  M   A  $     �  Q   ~  >    }  Q     �  �  N   A  $   �  �  Q   �  >  �  �  Q     �  �  O   A  $   �  �  Q   �  >  �  �  Q     �  �  P   A  $   �  �  Q   �  >  �  �  Q     �  �  Q   A  $   �  �  Q   �  >  �  �  Q     �  �  R   A  $   �  �  Q   �  >  �  �  Q     �  �  S   A  $   �  �  Q   �  >  �  �  Q     �  �  T   A  $   �  �  Q   �  >  �  �  Q     �  �  U   A  $   �  �  Q   �  >  �  �  Q     �  �  V   A  $   �  �  Q   �  >  �  �  Q     �  �  W   A  $   �  �  Q   �  >  �  �  Q     �  �  X   A  $   �  �  Q   �  >  �  �  Q     �  �  Y   A  $   �  �  Q   �  >  �  �  Q     �  �  Z   A  $   �  �  Q   �  >  �  �  Q     �  �  [   A  $   �  �  Q   �  >  �  �  Q     �  �  \   A  $   �  �  Q   �  >  �  �  Q     �  �  ]   A  $   �  �  Q   �  >  �  �  Q     �  �  ^   A  $   �  �  Q   �  >  �  �  Q     �  �  _   A  $   �  �  Q   �  >  �  �  Q     �  �  `   A  $   �  �  Q   �  >  �  �  Q     �  �  a   A  $   �  �  Q   �  >  �  �  Q     �  �  b   A  $   �  �  Q   �  >  �  �  Q     �  �  c   A  $   �  �  Q   �  >  �  �  Q     �  �  d   A  $   �  �  Q   �  >  �  �  Q     �  �  e   A  $   �  �  Q   �  >  �  �  Q     �  �  f   A  $   �  �  Q   �  >  �  �  Q     �  �  g   A  $   �  �  Q   �  >  �  �  Q     �  �  h   A  $   �  �  Q   �  >  �  �  Q     �  �  i   A  $   �  �  Q   �  >  �  �  Q     �  �  j   A  $   �  �  Q   �  >  �  �  Q     �  �  k   A  $   �  �  Q   �  >  �  �  Q     �  �  l   A  $   �  �  Q   �  >  �  �  Q     �  �  m   A  $   �  �  Q   �  >  �  �  Q     �  �  n   A  $   �  �  Q   �  >  �  �  Q     �  �  o   A  $   �  �  Q   �  >  �  �  Q     �  �  p   A  $   �  �  Q   �  >  �  �  Q     �  �  q   A  $   �  �  Q   �  >  �  �  Q     �  �  r   A  $   �  �  Q   �  >  �  �  Q     �  �  s   A  $   �  �  Q   �  >  �  �  Q     �  �  t   A  $   �  �  Q   �  >  �  �  Q     �  �  u   A  $   �  �  Q   �  >  �  �  Q     �  �  v   A  $   �  �  Q   �  >  �  �  Q     �  �  w   A  $   �  �  Q   �  >  �  �  Q     �  �  x   A  $      �  Q   �  >     �  Q       �  y   A  $     �  Q     >      Q       �  z   A  $     �  Q     >      Q       �  {   A  $   	  �  Q     >  	    Q     
  �  |   A  $     �  Q     >    
  Q       �  }   A  $     �  Q     >      Q       �  ~   A  $     �  Q     >      Q       �     A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $   !  �  Q      >  !    Q     "  �  �   A  $   $  �  Q   #  >  $  "  Q     %  �  �   A  $   '  �  Q   &  >  '  %  Q     (  �  �   A  $   *  �  Q   )  >  *  (  Q     +  �  �   A  $   -  �  Q   ,  >  -  +  Q     .  �  �   A  $   0  �  Q   /  >  0  .  Q     1  �  �   A  $   3  �  Q   2  >  3  1  Q     4  �  �   A  $   6  �  Q   5  >  6  4  Q     7  �  �   A  $   9  �  Q   8  >  9  7  Q     :  �  �   A  $   <  �  Q   ;  >  <  :  Q     =  �  �   A  $   ?  �  Q   >  >  ?  =  Q     @  �  �   A  $   B  �  Q   A  >  B  @  Q     C  �  �   A  $   E  �  Q   D  >  E  C  Q     F  �  �   A  $   H  �  Q   G  >  H  F  Q     I  �  �   A  $   K  �  Q   J  >  K  I  Q     L  �  �   A  $   N  �  Q   M  >  N  L  Q     O  �  �   A  $   Q  �  Q   P  >  Q  O  Q     R  �  �   A  $   T  �  Q   S  >  T  R  Q     U  �  �   A  $   W  �  Q   V  >  W  U  Q     X  �  �   A  $   Z  �  Q   Y  >  Z  X  Q     [  �  �   A  $   ]  �  Q   \  >  ]  [  Q     ^  �  �   A  $   `  �  Q   _  >  `  ^  Q     a  �  �   A  $   c  �  Q   b  >  c  a  Q     d  �  �   A  $   f  �  Q   e  >  f  d  Q     g  �  �   A  $   i  �  Q   h  >  i  g  Q     j  �  �   A  $   l  �  Q   k  >  l  j  Q     m  �  �   A  $   o  �  Q   n  >  o  m  Q     p  �  �   A  $   r  �  Q   q  >  r  p  Q     s  �  �   A  $   u  �  Q   t  >  u  s  Q     v  �  �   A  $   x  �  Q   w  >  x  v  Q     y  �  �   A  $   {  �  Q   z  >  {  y  Q     |  �  �   A  $   ~  �  Q   }  >  ~  |  Q       �  �   A  $   �  �  Q   �  >  �    Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q        �  �   A  $     �  Q     >       Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q     	  �  �   A  $     �  Q   
  >    	  Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $     �  Q     >      Q       �  �   A  $      �  Q     >       Q     !  �  �   A  $   #  �  Q   "  >  #  !  Q     $  �  �   A  $   &  �  Q   %  >  &  $  Q     '  �  �   A  $   )  �  Q   (  >  )  '  Q     *  �  �   A  $   ,  �  Q   +  >  ,  *  Q     -  �  �   A  $   /  �  Q   .  >  /  -  Q     0  �  �   A  $   2  �  Q   1  >  2  0  Q     3  �  �   A  $   5  �  Q   4  >  5  3  Q     6  �  �   A  $   8  �  Q   7  >  8  6  Q     9  �  �   A  $   ;  �  Q   :  >  ;  9  Q     <  �  �   A  $   >  �  Q   =  >  >  <  Q     ?  �  �   A  $   A  �  Q   @  >  A  ?  Q     B  �  �   A  $   D  �  Q   C  >  D  B  Q     E  �  �   A  $   G  �  Q   F  >  G  E  Q     H  �  �   A  $   J  �  Q   I  >  J  H  Q     K  �  �   A  $   M  �  Q   L  >  M  K  Q     N  �  �   A  $   P  �  Q   O  >  P  N  Q     Q  �  �   A  $   S  �  Q   R  >  S  Q  Q     T  �  �   A  $   V  �  Q   U  >  V  T  Q     W  �  �   A  $   Y  �  Q   X  >  Y  W  Q     Z  �  �   A  $   \  �  Q   [  >  \  Z  Q     ]  �  �   A  $   _  �  Q   ^  >  _  ]  Q     `  �  �   A  $   b  �  Q   a  >  b  `  Q     c  �  �   A  $   e  �  Q   d  >  e  c  Q     f  �  �   A  $   h  �  Q   g  >  h  f  Q     i  �  �   A  $   k  �  Q   j  >  k  i  Q     l  �  �   A  $   n  �  Q   m  >  n  l  Q     o  �  �   A  $   q  �  Q   p  >  q  o  Q     r  �  �   A  $   t  �  Q   s  >  t  r  Q     u  �  �   A  $   w  �  Q   v  >  w  u  Q     x  �  �   A  $   z  �  Q   y  >  z  x  Q     {  �  �   A  $   }  �  Q   |  >  }  {  Q     ~  �  �   A  $   �  �  Q     >  �  ~  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �  >  �  �  Q     �  �  �   A  $   �  �  Q   �   >  �  �  �  �  �  �  �  W   �  W   �  8                                                  HistogramData               histogram_data         �                            AverageHistogramConfig              AverageHistogramConfig 	       "       AverageHistogramConfig.pixel_count    #       AverageHistogramConfig.min_log_luma   %       AverageHistogramConfig.log_luma_range          AverageHistogramConfig.dt   %       AverageHistogramConfig.low_percentile   &       AverageHistogramConfig.high_percentile   '       AverageHistogramConfig.low_adjust_speed   (       AverageHistogramConfig.high_adjust_speed   )       AverageHistogramConfig.write_debug_output                                 HistogramResultBuffer              histogram_result                                      DebugOutput              debug_output                         main                                            HistogramData               histogram_data           �                            AverageHistogramConfig              AverageHistogramConfig 	       "       AverageHistogramConfig.pixel_count    #       AverageHistogramConfig.min_log_luma   %       AverageHistogramConfig.log_luma_range          AverageHistogramConfig.dt   %       AverageHistogramConfig.low_percentile   &       AverageHistogramConfig.high_percentile   '       AverageHistogramConfig.low_adjust_speed   (       AverageHistogramConfig.high_adjust_speed   )       AverageHistogramConfig.write_debug_output     0                                HistogramResultBuffer              histogram_result                                        DebugOutput              debug_output                                                                   HistogramData               histogram_data         �                            AverageHistogramConfig              AverageHistogramConfig 	       "       AverageHistogramConfig.pixel_count    #       AverageHistogramConfig.min_log_luma   %       AverageHistogramConfig.log_luma_range          AverageHistogramConfig.dt   %       AverageHistogramConfig.low_percentile   &       AverageHistogramConfig.high_percentile   '       AverageHistogramConfig.low_adjust_speed   (       AverageHistogramConfig.high_adjust_speed   )       AverageHistogramConfig.write_debug_output                                 HistogramResultBuffer              histogram_result                                      DebugOutput              debug_output                         main                                            HistogramData               histogram_data           �                            AverageHistogramConfig              AverageHistogramConfig 	       "       AverageHistogramConfig.pixel_count    #       AverageHistogramConfig.min_log_luma   %       AverageHistogramConfig.log_luma_range          AverageHistogramConfig.dt   %       AverageHistogramConfig.low_percentile   &       AverageHistogramConfig.high_percentile   '       AverageHistogramConfig.low_adjust_speed   (       AverageHistogramConfig.high_adjust_speed   )       AverageHistogramConfig.write_debug_output     0                                HistogramResultBuffer              histogram_result                                        DebugOutput              debug_output                                                                   HistogramData               histogram_data         �                            AverageHistogramConfig              AverageHistogramConfig 	       "       AverageHistogramConfig.pixel_count    #       AverageHistogramConfig.min_log_luma   %       AverageHistogramConfig.log_luma_range          AverageHistogramConfig.dt   %       AverageHistogramConfig.low_percentile   &       AverageHistogramConfig.high_percentile   '       AverageHistogramConfig.low_adjust_speed   (       AverageHistogramConfig.high_adjust_speed   )       AverageHistogramConfig.write_debug_output                                 HistogramResultBuffer              histogram_result                                      DebugOutput              debug_output                         main                                            HistogramData               histogram_data           �                            AverageHistogramConfig              AverageHistogramConfig 	       "       AverageHistogramConfig.pixel_count    #       AverageHistogramConfig.min_log_luma   %       AverageHistogramConfig.log_luma_range          AverageHistogramConfig.dt   %       AverageHistogramConfig.low_percentile   &       AverageHistogramConfig.high_percentile   '       AverageHistogramConfig.low_adjust_speed   (       AverageHistogramConfig.high_adjust_speed   )       AverageHistogramConfig.write_debug_output     0                                HistogramResultBuffer              histogram_result                                        DebugOutput              debug_output                            luma_average_histogram.comp