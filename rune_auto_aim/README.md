# rune_auto_aim
 ������rune_detector
 ������rune_shooter
 ������rune_tracker
 �������ؽ��ٶȺ���:
$$  w = A * cos (\omega * t + \phi) + b  $$

## rune_detector
��������ʶ�𣬻���������yolox������Ŀ�������磬ʹ��openvino������С�������ú��Լ�������
����������Ϊδ�����Ҷ���Ѽ����Ҷ��R��������������͸��ʣ��������ĵ�����pnp_solver���н��������ת��ƽ������

���ģ�
- ��������ڵ��ͼƬ `/image_for_rune`

������
- ���ս�����������ݷ��� `/detector/runes`

������
- ���������
  - ���Ŷ���ֵ confidence_threshold

- ������ѵ���ֿ⴫����
[�򺽿�Դ����](https://github.com/tup-robomaster/TUP-NN-Train-2)
  - [�Ͻ�ѵ������עת������](https://github.com/Spphire/RM-labeling-tool)
  �Ͻ�ת�����߻���Ҫ�Լ��޸�һ�²���ϸ��
- ��Ҷ��R��ı�ע��˳������ͼ
![](pic/RunePoint.jpg)

## rune_tracker
��������׷��ģ��,��Ҫͨ��ǰ��ļ��ģ��ɼ����ٶ����ݣ�Ȼ��������ukf�˲��������˲�����,���ƽ���Ľ��ٶ���������ceres�������
�������������ת���ٶ����Ǻ�������(��ֵ����λ����Ƶ�ʡ�����λ��)

### UKF�޼��������˲���
[�޼����������Ϳ�](https://github.com/saishiva024/LIDAR-RADAR-Fusion-UKF/blob/master/src/ukf.cpp)
[ukf��ʽ](https://zhuanlan.zhihu.com/p/359811364)
![](../armor_auto_aim/armor_tracker/docs/Kalman_filter_model.png)

### Ceres-solver
����Ŀʹ����С���˷����ȥ������Ǻ�������
[ceres-solver's github](https://github.com/ceres-solver/ceres-solver)
[��С���˷����Ϳ�](https://zhuanlan.zhihu.com/p/38128785)
