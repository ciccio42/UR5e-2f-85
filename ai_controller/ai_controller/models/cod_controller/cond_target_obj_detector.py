import torch.multiprocessing as mp
import copy
import torch
import torch.nn as nn
import numpy as np
import torch.nn.functional as F
from einops import rearrange, parse_shape
import os
import hydra
# mmdet moduls
from torchsummary import summary
from .utils import *
import torch
from torch.autograd import Variable
from torchvision import ops
from torchvision.models.video import r2plus1d_18, R2Plus1D_18_Weights
import cv2
import matplotlib.pyplot as plt
import time

DEBUG = False
def get_backbone(backbone_name="slow_r50", video_backbone=True, pretrained=False, conv_drop_dim=3):
    if video_backbone:
        print(f"Loading video backbone {backbone_name}.....")
        if backbone_name == "r2plus1d_18":
            if not pretrained:
                weights = None
            else:
                weights = R2Plus1D_18_Weights

            return nn.Sequential(*list(r2plus1d_18(weights=weights).children())[:-1])
    else:
        print(f"Loading  backbone {backbone_name}.....")
        if backbone_name == "resnet18":
            return ResNetFeats(use_resnet18=True,
                               pretrained=pretrained,
                               output_raw=True,
                               drop_dim=conv_drop_dim)


def conv(ic, oc, k, s, p):
    return nn.Sequential(
        nn.Conv2d(ic, oc, k, s, p),
        nn.ReLU(inplace=True),
        nn.BatchNorm2d(oc),
    )


def coord_map(shape, start=-1, end=1, gpu_id=0):
    """
    Gives, a 2d shape tuple, returns two mxn coordinate maps,
    Ranging min-max in the x and y directions, respectively.
    """
    m, n = shape
    x_coord_row = torch.linspace(
        start, end, steps=n).to(device=gpu_id).type(torch.cuda.FloatTensor)
    y_coord_row = torch.linspace(
        start, end, steps=m).to(device=gpu_id).type(torch.cuda.FloatTensor)
    x_coords = x_coord_row.unsqueeze(0).expand(torch.Size((m, n))).unsqueeze(0)
    y_coords = y_coord_row.unsqueeze(1).expand(torch.Size((m, n))).unsqueeze(0)
    return Variable(torch.cat([x_coords, y_coords], 0))


class FeatureExtractor(nn.Module):
    def __init__(self):
        super(FeatureExtractor, self).__init__()

        self.model = nn.Sequential(
            # conv(3, 128, 5, 1, 2),
            # conv(128, 128, 3, 1, 1),
            # conv(128, 128, 4, 2, 1),
            # conv(128, 128, 4, 2, 1),
            # conv(128, 128, 4, 2, 1),
            conv(3, 128, 5, 2, 2),
            conv(128, 128, 3, 2, 1),
            conv(128, 128, 3, 2, 1),
            conv(128, 128, 3, 1, 1),
            conv(128, 128, 3, 1, 1),
            # conv(3, 128, 4, 2, 2),
            # conv(128, 128, 4, 2, 1),
            # conv(128, 128, 4, 2, 1),
            # conv(128, 128, 4, 2, 1),
        )

    def forward(self, x):
        return self.model(x)


class FiLMBlock(nn.Module):
    def __init__(self):
        super(FiLMBlock, self).__init__()

    def forward(self, x, gamma, beta):
        beta = beta.view(x.size(0), x.size(1), 1, 1)
        gamma = gamma.view(x.size(0), x.size(1), 1, 1)

        x = gamma * x + beta

        return x


class ResBlock(nn.Module):
    def __init__(self, in_place, out_place):
        super(ResBlock, self).__init__()

        self.conv1 = nn.Conv2d(in_place, out_place, 1, 1, 0)
        self.relu1 = nn.ReLU(inplace=True)

        self.conv2 = nn.Conv2d(out_place, out_place, 3, 1, 1)
        self.norm2 = nn.BatchNorm2d(out_place)
        self.film = FiLMBlock()
        self.relu2 = nn.ReLU(inplace=True)

    def forward(self, x, beta, gamma):
        x = self.conv1(x)
        x = self.relu1(x)
        identity = x

        x = self.conv2(x)
        x = self.norm2(x)
        x = self.film(x, beta, gamma)
        x = self.relu2(x)

        x = x + identity

        return x


class ClassificationModule(nn.Module):
    def __init__(self, out_channels, n_classes, roi_size, hidden_dim=512, p_dropout=0.3):
        super().__init__()
        self.roi_size = roi_size
        # hidden network
        self.avg_pool = nn.AvgPool2d(self.roi_size)
        self.fc = nn.Linear(out_channels, hidden_dim)
        self.dropout = nn.Dropout(p_dropout, inplace=True)

        # define classification head
        self.cls_head = nn.Linear(hidden_dim, n_classes)

    def forward(self, feature_map, proposals_list, gt_classes=None):

        # if gt_classes is None:
        #     mode = 'eval'
        # else:
        #     mode = 'train'

        # apply roi pooling on proposals followed by avg pooling
        roi_out = ops.roi_pool(feature_map, proposals_list, self.roi_size)
        roi_out = self.avg_pool(roi_out)

        # flatten the output
        roi_out = roi_out.squeeze(-1).squeeze(-1)

        # pass the output through the hidden network
        out = self.fc(roi_out)
        # Number of positive bb, Embedding size
        out = F.relu(self.dropout(out))

        # get the classification scores
        cls_scores = self.cls_head(out)  # Number of positive bb, Num classes

        return cls_scores


class FiLM(nn.Module):
    def __init__(self, backbone_name="resnet18", conv_drop_dim=3, n_res_blocks=18, n_classes=1, n_channels=128, task_embedding_dim=128, pretrained=False):
        super(FiLM, self).__init__()

        self.task_embedding_dim = task_embedding_dim

        self.film_generator = nn.Linear(
            task_embedding_dim, 2 * n_res_blocks * n_channels)
        self.feature_extractor = get_backbone(backbone_name=backbone_name,
                                              video_backbone=False,
                                              pretrained=pretrained,
                                              conv_drop_dim=conv_drop_dim)
        self.res_blocks = nn.ModuleList()

        for _ in range(n_res_blocks):
            self.res_blocks.append(ResBlock(n_channels + 2, n_channels))

        self.n_res_blocks = n_res_blocks
        self.n_channels = n_channels

    def forward(self, agent_obs, task_emb):

        sizes = parse_shape(agent_obs, 'B T _ _ _')

        agent_obs = rearrange(agent_obs, "B T C H W -> (B T) C H W")
        agent_obs_feat = self.feature_extractor(agent_obs)  # B*T, C, H, W
        # agent_obs_feat = rearrange(
        #     agent_obs_feat, "(B T) C H W -> B T C H W", **sizes)

        film_vector = self.film_generator(task_emb).view(
            sizes['B'], self.n_res_blocks, 2, self.n_channels)  # B N_RES 2(alpha, beta) N_CHANNELS

        # B*T N_RES 2(alpha, beta) N_CHANNELS
        film_vector = film_vector.repeat_interleave(sizes['T'], 0)

        h = agent_obs_feat.size(2)
        w = agent_obs_feat.size(3)
        coords = coord_map(shape=(h, w),
                           gpu_id=agent_obs_feat.get_device())[None].repeat(
            agent_obs_feat.shape[0], 1, 1, 1).to(agent_obs_feat.get_device())  # B 2 h w

        for i, res_block in enumerate(self.res_blocks):
            beta = film_vector[:, i, 0, :]
            gamma = film_vector[:, i, 1, :]

            if i == 0:
                x = agent_obs_feat
            x = torch.cat([x, coords], 1)
            x = res_block(x, beta, gamma)

        cond_feature = x

        return cond_feature


class ProposalModule(nn.Module):
    def __init__(self, in_features, hidden_dim=512, n_anchors=9, p_dropout=0.3):
        super().__init__()
        self.n_anchors = n_anchors
        self.conv1 = nn.Conv2d(in_features, hidden_dim,
                               kernel_size=3, padding=1)
        self.dropout = nn.Dropout(p_dropout, inplace=True)
        self.conf_head = nn.Conv2d(hidden_dim, n_anchors, kernel_size=1)
        self.reg_head = nn.Conv2d(hidden_dim, n_anchors * 4, kernel_size=1)

    def forward(self, feature_map, pos_anc_ind=None, neg_anc_ind=None, pos_anc_coords=None):
        # determine mode
        if pos_anc_ind is None or neg_anc_ind is None or pos_anc_coords is None:
            mode = 'eval'
            self.dropout.training = False
        else:
            mode = 'train'
            self.dropout.training = True

        out = self.conv1(feature_map)
        out = self.dropout(out)
        out = F.relu(out, inplace=True)

        # for each image and for each anchor box the head produces
        reg_offsets_pred = self.reg_head(out)  # (B, A*4, hmap, wmap)
        conf_scores_pred = self.conf_head(out)  # (B, A, hmap, wmap)

        if mode == 'train':
            # get conf scores
            conf_scores_pos = conf_scores_pred.flatten()[pos_anc_ind]
            conf_scores_neg = conf_scores_pred.flatten()[neg_anc_ind]
            # get offsets for +ve anchors
            offsets_pos = reg_offsets_pred.contiguous().view(-1,
                                                             4)[pos_anc_ind]
            # generate proposals using offsets
            proposals = generate_proposals(pos_anc_coords, offsets_pos)

            return conf_scores_pos, conf_scores_neg, offsets_pos, proposals

        elif mode == 'eval':
            return conf_scores_pred, reg_offsets_pred


def make_model(model_dict, backbone_name="resnet18", task_embedding_dim=128, conv_drop_dim=3, pretrained=False):
    backbone = FiLM(
        backbone_name=backbone_name,
        conv_drop_dim=conv_drop_dim,
        n_res_blocks=model_dict['n_res_blocks'],
        n_classes=model_dict['n_classes'],
        n_channels=model_dict['n_channels'],
        task_embedding_dim=task_embedding_dim,
        pretrained=pretrained)
    # for name, module in backbone.named_children():
    #     if name == "res_blocks":
    #         for name, module in backbone.res_blocks.named_children():
    #             if name == "5":
    #                 for name, module in backbone.res_blocks.name.named_children():
    #                     print(name, module)
    return backbone


class CondModule(nn.Module):

    def __init__(self, height=120, width=160, demo_T=4, model_name="slow_r50", pretrained=False, cond_video=True, n_layers=3, demo_W=7, demo_H=7, demo_ff_dim=[128, 64, 32], demo_linear_dim=[512, 256, 128], conv_drop_dim=3):
        super().__init__()
        self._demo_T = demo_T
        self._cond_video = cond_video

        self._backbone = get_backbone(backbone_name=model_name,
                                      video_backbone=cond_video,
                                      pretrained=pretrained,
                                      conv_drop_dim=conv_drop_dim)

        if not cond_video:
            conv_layer = []
            if conv_drop_dim == 2:
                input_dim_0 = 512
            elif conv_drop_dim == 3:
                input_dim_0 = 256
            input_dim = [input_dim_0, demo_ff_dim[0], demo_ff_dim[1]]
            output_dim = demo_ff_dim
            for i in range(n_layers):
                if i == 0:
                    depth_dim = self._demo_T
                else:
                    depth_dim = 1
                conv_layer.append(
                    nn.Conv3d(input_dim[i], output_dim[i], (depth_dim, 1, 1), bias=False))

            self._3d_conv = nn.Sequential(*conv_layer)
            linear_input = demo_ff_dim[-1] * demo_W * demo_H
        else:
            # [TODO] Implement ff for video backbone
            linear_input = 512

        # MLP encoder
        mlp_encoder = []
        for indx, layer_dim in enumerate(demo_linear_dim):
            if indx == 0:
                input_dim = linear_input
            else:
                input_dim = demo_linear_dim[indx-1]
            mlp_encoder.append(nn.Linear(in_features=input_dim,
                                         out_features=layer_dim))
            mlp_encoder.append(nn.ReLU())

        self._mlp_encoder = nn.Sequential(*mlp_encoder)

    def forward(self, input):
        # 1. Compute features for each frame in the batch
        sizes = parse_shape(input, 'B T _ _ _')
        if not self._cond_video:
            backbone_input = rearrange(input, 'B T C H W -> (B T) C H W')
            backbone_out = self._backbone(backbone_input)
            backbone_out = rearrange(
                backbone_out, '(B T) C H W -> B T C H W', **sizes)
            backbone_out = rearrange(backbone_out, 'B T C H W -> B C T H W')
            temp_conv_out = self._3d_conv(backbone_out)
            temp_conv_out = rearrange(temp_conv_out, 'B C T H W -> B T C H W')
            linear_input = torch.flatten(temp_conv_out, start_dim=1)
            task_embedding = self._mlp_encoder(linear_input)
        else:
            backbone_input = rearrange(input, 'B T C H W -> B C T H W')
            backbone_out = rearrange(self._backbone(
                backbone_input), 'B C T H W -> B (C T H W)')
            task_embedding = self._mlp_encoder(backbone_out)

        # print(task_embedding.shape)
        return task_embedding


class AgentModule(nn.Module):

    def __init__(self, height=120, width=160, obs_T=4, model_name="resnet18", pretrained=False, load_film=True, n_res_blocks=6, n_classes=2, task_embedding_dim=128, dim_H=7, dim_W=7, conv_drop_dim=3, anc_scales=[1.0, 1.5, 2.0, 3.0, 4.0], anc_ratios=[0.2, 0.5, 0.8, 1, 1.2, 1.5, 2.0], x_offset=1.5, y_offset=1.5):
        super().__init__()
        self.task_embedding_dim = task_embedding_dim
        if not load_film:
            self._module = get_backbone(backbone_name=model_name,
                                        video_backbone=False,
                                        pretrained=pretrained,
                                        conv_drop_dim=conv_drop_dim)
        else:
            #### Create model with backbone + film ####
            model_dict = dict()
            model_dict['n_res_blocks'] = n_res_blocks
            model_dict['n_classes'] = n_classes
            if conv_drop_dim == 3:
                n_channels = 256
            elif conv_drop_dim == 2:
                n_channels = 512
            else:
                n_channels = 128
            model_dict['n_channels'] = n_channels
            backbone = make_model(model_dict=model_dict,
                                  task_embedding_dim=task_embedding_dim,
                                  conv_drop_dim=conv_drop_dim,
                                  pretrained=pretrained)
            backbone.out_channels = n_channels
            self.out_channels_backbone = n_channels
            self._backbone = backbone
            print("---- Summary backbone with FiLM layer ----")
            # summary(self._backbone)

            #### Create Region Proposal Network ####
            self.img_height, self.img_width = height, width
            self.out_h, self.out_w = dim_H, dim_W

            # downsampling scale factor
            self.width_scale_factor = self.img_width // self.out_w
            self.height_scale_factor = self.img_height // self.out_h

            # scales and ratios for anchor boxes
            self.anc_scales = anc_scales
            self.anc_ratios = anc_ratios
            self.n_anc_boxes = len(self.anc_scales) * len(self.anc_ratios)

            # IoU thresholds for +ve and -ve anchors
            self.pos_thresh = 0.4
            self.neg_thresh = 0.3
            self.conf_thresh = 0.7
            self.nms_thresh = 0.5

            self.proposal_module = ProposalModule(
                self.out_channels_backbone,
                n_anchors=self.n_anc_boxes)

            self.classifier = ClassificationModule(
                out_channels=self.out_channels_backbone,
                n_classes=n_classes,
                roi_size=(2, 2))

            # generate anchors
            start = time.time()
            self.anc_pts_x, self.anc_pts_y = gen_anc_centers(
                out_size=(self.out_h, self.out_w),
                x_offset=x_offset,
                y_offset=y_offset)
            print(f"Gen anc centers {time.time()-start}")

            start = time.time()
            self.anc_base = gen_anc_base(
                self.anc_pts_x, self.anc_pts_y, self.anc_scales, self.anc_ratios, (self.out_h, self.out_w))
            print(f"Gen_anc_base {time.time()-start}")

        self.load_film = load_film

    def visualize_attention(self, input_image, feature_map, save_path="attention_overlay.png"):
        # Step 1: Get the feature map
        # Assuming feature_map is of shape [batch_size, channels, height, width]
        # For visualization, you can take the mean across the channel dimension
        attention_map = torch.mean(feature_map, dim=1).squeeze()  # [height, width]

        # Step 2: Rescale attention map to the input image size
        attention_map_resized = F.interpolate(attention_map.unsqueeze(0).unsqueeze(0),
                                            size=(input_image.shape[-2], input_image.shape[-1]),
                                            mode='bilinear', align_corners=False).squeeze().cpu().detach().numpy()

        # Normalize the attention map between 0 and 1
        attention_map_resized = (attention_map_resized - np.min(attention_map_resized)) / (np.max(attention_map_resized) - np.min(attention_map_resized))

        # Step 3: Convert the input image to numpy
        input_image_np = 255*(input_image.squeeze().permute(1, 2, 0).cpu().detach().numpy())
        # Step 4: Apply colormap to the attention map
        heatmap = cv2.applyColorMap(np.uint8(255 * attention_map_resized), cv2.COLORMAP_JET)

        # Step 5: Overlay the heatmap on the input image
        overlay = 0.6 * input_image_np + 0.4 * heatmap  # Weighted sum for overlay

        # Step 6: Save the result
        cv2.imwrite(save_path, overlay)

    def forward(self, agent_obs, task_embedding, gt_bb=None, gt_classes=None, inference=False):

        ret_dict = dict()

        ret_dict['task_embedding'] = task_embedding
        # 1. Compute conditioned embedding
        feature_map = self._backbone(
            agent_obs=agent_obs, task_emb=task_embedding)

        # self.visualize_attention(input_image=agent_obs,
        #                          feature_map=feature_map)        
    
        # 2. Predict bounding boxes given conditioned embedding and input image
        agent_obs = rearrange(agent_obs, 'B T C H W -> (B T) C H W')
        # N is the number of objects, and C the bb components
        gt_bb = rearrange(gt_bb, 'B T N C -> (B T) N C')
        # N is the number of objects
        gt_classes = rearrange(gt_classes, 'B T N -> (B T) N')
        B, C, H, W = agent_obs.shape

        if self.load_film:

            if DEBUG:
                # # plot anchor boxes
                anc_pts_x_image = self.anc_pts_x.clone() * self.width_scale_factor
                anc_pts_y_image = self.anc_pts_y.clone() * self.height_scale_factor
                image = np.array(np.moveaxis(
                    agent_obs[0, :, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)
                for anc_x in anc_pts_x_image.numpy():
                    for anc_y in anc_pts_y_image.numpy():
                        image = cv2.circle(np.ascontiguousarray(image), (int(anc_x), int(anc_y)),
                                           radius=int(1), color=(0, 0, 255), thickness=1)
                cv2.imwrite("prova_anc_pts.png", image)

            anc_boxes_all = self.anc_base.repeat(
                B, 1, 1, 1, 1).to(agent_obs.get_device())  # B, Feature_H, Feature_W,

            if DEBUG:
                # # Plot anchor boxes to image
                for x, anc_x in enumerate(anc_pts_x_image.numpy()):
                    for y, anc_y in enumerate(anc_pts_y_image.numpy()):
                        anc_boxes_proj = rearrange(project_bboxes(
                            anc_boxes_all, self.width_scale_factor, self.height_scale_factor, mode='a2p'), 'B (X Y Z) C -> B X Y Z C',
                            X=anc_boxes_all.shape[1],
                            Y=anc_boxes_all.shape[2],
                            Z=anc_boxes_all.shape[3])
                        anc_boxes_proj_interest = anc_boxes_proj[0, x, y, :, :].cpu(
                        ).numpy()
                        image = np.array(np.moveaxis(
                            agent_obs[0, :, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)
                        image = cv2.rectangle(np.ascontiguousarray(image),
                                              (int(gt_bb[0, 0, 0]),
                                               int(gt_bb[0, 0, 1])),
                                              (int(gt_bb[0, 0, 2]),
                                               int(gt_bb[0, 0, 3])),
                                              color=(0, 0, 255), thickness=1)
                        for anc_box in anc_boxes_proj_interest:
                            image = cv2.rectangle(np.ascontiguousarray(image),
                                                  (int(anc_box[0]),
                                                   int(anc_box[1])),
                                                  (int(anc_box[2]),
                                                   int(anc_box[3])),
                                                  color=(0, 0, 255), thickness=1)
                cv2.imwrite("prova_anch_box.png", image)

            if not inference:
                # if the model is training
                # get positive and negative anchors amongst other things
                gt_bboxes_proj = project_bboxes(
                    gt_bb.float(),
                    torch.tensor(self.width_scale_factor, dtype=float),
                    torch.tensor(self.height_scale_factor, dtype=float),
                    mode='p2a').float()

                positive_anc_ind, negative_anc_ind, GT_conf_scores, GT_offsets, GT_class_pos, positive_anc_coords, negative_anc_coords, positive_anc_ind_sep = get_req_anchors(
                    anc_boxes_all.to(agent_obs.get_device()),
                    gt_bboxes_proj.to(agent_obs.get_device()),
                    gt_classes.to(agent_obs.get_device()),
                    pos_thresh=self.pos_thresh,
                    neg_thresh=self.neg_thresh)

                # pass through the proposal module
                conf_scores_pos, conf_scores_neg, offsets_pos, proposals = self.proposal_module(
                    feature_map,
                    positive_anc_ind,
                    negative_anc_ind,
                    positive_anc_coords)

                # get separate proposals for each sample
                pos_proposals_list = []
                # class_positive_list = []
                batch_size = B

                # Create a mask for each index in the batch
                batch_indices = torch.arange(batch_size, device=positive_anc_ind_sep.device).unsqueeze(1)
                mask = positive_anc_ind_sep.unsqueeze(0) == batch_indices

                # Use the mask to gather proposals and classes
                pos_proposals_list = [proposals[torch.where(mask[i])[0]].detach().clone() for i in range(batch_size)]
                # class_positive_list = [GT_class_pos[torch.where(mask[i])[0]].detach().clone() for i in range(batch_size)]
                # print(f"Time to separate proposals {time.time()-start_time}")
                
                cls_scores = self.classifier(
                    feature_map, pos_proposals_list, GT_class_pos)

                ret_dict['feature_map'] = feature_map
                ret_dict['proposals'] = pos_proposals_list
                ret_dict['GT_offsets'] = GT_offsets
                ret_dict['offsets_pos'] = offsets_pos
                ret_dict['conf_scores_pos'] = conf_scores_pos
                ret_dict['conf_scores_neg'] = conf_scores_neg
                ret_dict['cls_scores'] = cls_scores
                ret_dict['GT_class_pos'] = GT_class_pos

                # if DEBUG:
                #     # test plot proposal
                #     proposal_projected = project_bboxes(
                #         proposals, self.width_scale_factor, self.height_scale_factor, mode='a2p').cpu().detach().numpy()
                #     for indx, bb_proposal in enumerate(proposal_projected):
                #         image = np.array(np.moveaxis(
                #             agent_obs[indx, :, :, :].cpu().numpy()*255, 0, -1), dtype=np.uint8)
                #         image = cv2.rectangle(np.ascontiguousarray(image),
                #                               (int(bb_proposal[0]), int(
                #                                   bb_proposal[1])),
                #                               (int(bb_proposal[2]), int(
                #                                   bb_proposal[3])),
                #                               color=(0, 0, 255), thickness=1)
                #         cv2.imwrite("prova_predictions_eval.png", image)

                return ret_dict
            else:
                # model is in inference mode
                with torch.no_grad():
                    start = time.time()
                    anc_boxes_flat = anc_boxes_all.reshape(B, -1, 4)

                    # get conf scores and offsets
                    # start = time.time()
                    conf_scores_pred, offsets_pred = self.proposal_module(
                        feature_map)
                    # print(f"Proposal module {time.time()-start}")
                    conf_scores_pred = conf_scores_pred.reshape(B, -1)
                    offsets_pred = offsets_pred.reshape(B, -1, 4)

                    # filter out proposals based on conf threshold and nms threshold for each image

                    proposals_final = []
                    conf_scores_final = []

                    # Parallel
                    # # Process multiple images (B) in parallel (batch processing)
                    # conf_scores = torch.sigmoid(conf_scores_pred)
                    # offsets = offsets_pred
                    # anc_boxes = anc_boxes_flat

                    # # Generate proposals for the entire batch
                    # proposals = generate_proposals(anc_boxes.to(
                    #     agent_obs.get_device()), offsets.to(agent_obs.get_device()))

                    # # Filter based on confidence threshold for the entire batch
                    # conf_idx = torch.where(conf_scores >= self.conf_thresh)

                    # # Apply NMS for the entire batch
                    # proposal_pos = proposals[conf_idx]
                    # conf_scores_pos = conf_scores[conf_idx]
                    # nms_idx = ops.nms(
                    #     proposal_pos, conf_scores_pos, self.nms_thresh)

                    # # Extract filtered proposals and confidence scores
                    # # No need for list comprehension
                    # proposals_pos = proposal_pos[nms_idx]
                    # # No need for list comprehension
                    # conf_scores_pos = conf_scores_pos[nms_idx]

                    # # Classifier forward pass for the entire batch of proposals
                    # cls_scores = self.classifier(feature_map, proposals_pos)
                    # cls_probs = F.softmax(cls_scores, dim=-1)

                    # # Get classes with the highest probability for the entire batch
                    # classes_all = torch.argmax(cls_probs, dim=-1)

                    # # Slice classes to map to their corresponding images
                    # classes_final = []
                    # c = 0
                    # for i in range(B):
                    #     # Determine the number of proposals for this image
                    #     n_proposals = len(proposals_pos[i])
                    #     classes_final.append(classes_all[c: c + n_proposals])
                    #     c += n_proposals

                    # Sequential
                    for i in range(B):
                        conf_scores = torch.sigmoid(conf_scores_pred[i])
                        offsets = offsets_pred[i]
                        anc_boxes = anc_boxes_flat[i]
                        proposals = generate_proposals(
                            anc_boxes.to(agent_obs.get_device()),
                            offsets.to(agent_obs.get_device()))
                        # filter based on confidence threshold
                        conf_idx = torch.where(
                            conf_scores >= self.conf_thresh)[0]
                        conf_scores_pos = conf_scores[conf_idx]
                        proposals_pos = proposals[conf_idx]
                        # filter based on nms threshold
                        nms_idx = ops.nms(
                            proposals_pos, conf_scores_pos, self.nms_thresh)

                        proposals_pos = proposals_pos[nms_idx]
                        conf_scores_pos = conf_scores_pos[nms_idx]
                        proposals_final.append(proposals_pos)
                        conf_scores_final.append(conf_scores_pos)

                        # try:
                        #     max_indx = torch.argmax(
                        #         conf_scores_pos[nms_idx])
                        #     conf_scores_pos = conf_scores_pos[nms_idx][max_indx]
                        #     proposals_pos = proposals_pos[nms_idx][max_indx].float(
                        #     )
                        # except:
                        #     # print("No bb found")
                        #     conf_scores_pos = torch.tensor(
                        #         -1).to(agent_obs.get_device())
                        #     proposals_pos = torch.tensor(
                        #         [-1, -1, -1, -1]).to(
                        #         agent_obs.get_device()).float()
                        # proposals_final.append(proposals_pos)
                        # conf_scores_final.append(conf_scores_pos)
                    # print(f"Sequential {time.time()-start}")

                    cls_scores = self.classifier(feature_map, proposals_final)
                    cls_probs = F.softmax(cls_scores, dim=-1)
                    # get classes with highest probability
                    classes_all = torch.argmax(cls_probs, dim=-1)

                  
                    classes_final = []
                    # slice classes to map to their corresponding image
                    c = 0
                    for i in range(B):
                        # get the number of proposals for each image
                        n_proposals = len(proposals_final[i])
                        classes_final.append(classes_all[c: c+n_proposals])
                        c += n_proposals

                    # print(f"Inference time {time.time()-start}")
                    ret_dict['proposals'] = proposals_final
                    ret_dict['conf_scores_final'] = conf_scores_final
                    ret_dict['cls_scores'] = cls_scores
                    ret_dict['feature_map'] = feature_map
                    ret_dict['classes_final'] = classes_final

                    return ret_dict

        else:
            scene_embegging = self._module(input)
            # print(scene_embegging.shape)
            return self._module(input)


class CondTargetObjectDetector(nn.Module):

    def __init__(self,
                 cond_target_obj_detector_cfg):
        super().__init__()

        self.dim_H = cond_target_obj_detector_cfg.dim_H
        self.dim_W = cond_target_obj_detector_cfg.dim_W

        self._cond_backbone = CondModule(height=cond_target_obj_detector_cfg.height,
                                         width=cond_target_obj_detector_cfg.width,
                                         demo_T=cond_target_obj_detector_cfg.demo_T,
                                         model_name=cond_target_obj_detector_cfg.cond_backbone_name,
                                         pretrained=cond_target_obj_detector_cfg.pretrained,
                                         cond_video=cond_target_obj_detector_cfg.cond_video,
                                         demo_H=cond_target_obj_detector_cfg.dim_H,
                                         demo_W=cond_target_obj_detector_cfg.dim_W,
                                         conv_drop_dim=cond_target_obj_detector_cfg.conv_drop_dim,
                                         demo_ff_dim=cond_target_obj_detector_cfg.demo_ff_dim,
                                         demo_linear_dim=cond_target_obj_detector_cfg.demo_linear_dim
                                         )

        self._agent_backone = AgentModule(height=cond_target_obj_detector_cfg.height,
                                          width=cond_target_obj_detector_cfg.width,
                                          obs_T=cond_target_obj_detector_cfg.obs_T,
                                          model_name=cond_target_obj_detector_cfg.agent_backbone_name,
                                          pretrained=cond_target_obj_detector_cfg.pretrained,
                                          dim_H=cond_target_obj_detector_cfg.dim_H,
                                          dim_W=cond_target_obj_detector_cfg.dim_W,
                                          conv_drop_dim=cond_target_obj_detector_cfg.conv_drop_dim,
                                          task_embedding_dim=cond_target_obj_detector_cfg.task_embedding_dim,
                                          anc_ratios=cond_target_obj_detector_cfg.anc_ratios,
                                          anc_scales=cond_target_obj_detector_cfg.anc_scales,
                                          n_classes=cond_target_obj_detector_cfg.get('n_classes', 2),
                                          x_offset=cond_target_obj_detector_cfg.x_offset,
                                          y_offset=cond_target_obj_detector_cfg.y_offset,)

        # summary(self)
        model_parameters = filter(lambda p: p.requires_grad, self.parameters())
        params = sum([np.prod(p.size()) for p in model_parameters])
        print('Total params in Detection module:', params)

        # Initialize hooks
        self.activations = None
        self.gradients = None

    def forward(self, inputs: list, inference: bool = False):
        cond_video = inputs[0] # B, T_demo, C, H, W
        agent_obs = inputs[1] # B, T_frame, C, H, W
        gt_bb = inputs[2]
        gt_classes = inputs[3]

        cond_emb = self._cond_backbone(cond_video)
        # print(f"Cond embedding shape: {cond_emb.shape}")
        ret_dict = self._agent_backone(
            agent_obs,
            cond_emb,
            gt_bb=gt_bb,
            gt_classes=gt_classes,
            inference=inference)
        # print(agent_emb.shape)
        return ret_dict

    def activations_hook(self, grad):
        self.activations = grad

    def register_forward_hook(self, layer):
        """
        Register a forward hook on the specified layer.
        """
        layer.register_forward_hook(self.activations_hook)

    def gradients_hook(self, module, grad_input, grad_output):
        self.gradients = grad_output[0]

    def register_backward_hook(self, layer):
        """
        Register a backward hook on the specified layer.
        """
        layer.register_backward_hook(self.gradients_hook)

    def get_activations(self, x):
        return self.activations

    def get_activations_gradient(self):
        return self.gradients

    def get_scale_factors(self):
        return [self._agent_backone.width_scale_factor, self._agent_backone.height_scale_factor]


@hydra.main(
    version_base=None,
    config_path="../../../experiments",
    config_name="config_cond_target_obj_detector.yaml")
def main(cfg):
    import debugpy
    debugpy.listen(('0.0.0.0', 5678))
    print("Waiting for debugger attach")
    debugpy.wait_for_client()
    inputs = dict()

    width = 100
    height = 180
    demo_T = 4
    inputs['demo'] = torch.rand(
        (4, 3, height, width),  dtype=torch.float).to('cuda:0')[None]  # B, T, C, W, H

    inputs['images'] = torch.rand(
        (1, 3, height, width),  dtype=torch.float).to('cuda:0')[None]  # B, T, C, W, H

    inputs['gt_bb'] = torch.rand(
        (1, 1, 1, 4),  dtype=torch.float).to('cuda:0')[None]

    inputs['gt_classes'] = torch.rand(
        (1, 1, 1, 1),  dtype=torch.float).to('cuda:0')[None]

    cfg.cond_target_obj_detector_cfg.conv_drop_dim = 4
    module = CondTargetObjectDetector(
        cond_target_obj_detector_cfg=cfg.cond_target_obj_detector_cfg)

    module.to('cuda:0')
    module(inputs, inference=True)


if __name__ == '__main__':
    main()
