这些模型来自用户提供的 mujoco_models.zip 中已转换的 rm21_armor_module：
small_armor、small_armor_target、small_armor_light_bar、small_num2_plane 和 small_num2.png。
只导入四板靶标使用的资产，原网格与贴图保持不变。源坐标是米，+X 朝板面前方，
原板底原点到中心的 Z 偏移为 0.0625 m，装配时将原点移到板中心。

外观使用 mesh；命中碰撞使用单独薄盒，几何体名称为 armor_hitbox_0..3。
用户上传包未提供单独的资产许可证；保留来源记录，不将它们视为项目自创资产。
