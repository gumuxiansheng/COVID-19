# 武汉市疫情定点医院信息

- hospitals-wuhan.json: 武汉市定点医院，来源腾讯疫情网站（https://card.wecity.qq.com/feverHosp/feverHospList?cityCode=420100&pageIndex=1&pageSize=999&partnerType=4&lat=0&lng=0&searchKey=）
- fangcang-hospitals.json: 武汉市方舱医院，来源腾讯地图搜索（https://apis.map.qq.com/ws/place/v1/search?keyword=%E6%96%B9%E8%88%B1&boundary=region(%E6%AD%A6%E6%B1%89,0)&page_size=20&key=\*\*\*\*-\*\*\*\*-\*\*\*\*-\*\*\*\*-\*\*\*\*-\*\*\*\*）
- huoshen-leishen-hospital.json: 火神山、雷神山，来源腾讯地图搜索（https://apis.map.qq.com/ws/place/v1/search?keyword=\*\*&boundary=region(%E6%AD%A6%E6%B1%89,0)&page_size=20&key=\*\*\*\*-\*\*\*\*-\*\*\*\*-\*\*\*\*-\*\*\*\*-\*\*\*\*）
- 武汉市红十字会（https://baijiahao.baidu.com/s?id=1657349545919501745&wfr=spider&for=pc）、武汉市国际博览中心临时仓库（https://baijiahao.baidu.com/s?id=1657265539595637723&wfr=spider&for=pc）、中华慈善总会（蒙牛）疫情防控应急物资中心（http://www.hbcf.org.cn/nv.html?nid=f69894c8-1bca-442a-9df1-31ecababa52e）、武汉市江夏区应急管理局（http://yjt.hubei.gov.cn/yjgl/ztzl/xxgzbdfk/yqdt/202003/t20200304_2172350.shtml）

运输车辆：20吨

## 数据注释

地点分为5类：

1. 仓库（Storehouse）
2. 方舱医院（Fangcang）
3. 火神山、雷神山医院（HuoshenLeishen）
4. 定点收治医院（Core）
5. 发热门诊医院（Fever）

* XXX_distance.csv(XXX='Core' or 'Fangcang' or 'Fever' or 'HuoshenLeishen')：仓库到各个地点的距离数据。
* XXX_distance.csv(XXX='Core' or 'Fangcang' or 'Fever' or 'HuoshenLeishen')：仓库到各个地点的行车时间（根据路况而定，随数据采集时间的不同而会有变化）。
* hospitals.csv：武汉疫情相关医院地址信息。
* locations.csv：医院及仓库的地址信息。
